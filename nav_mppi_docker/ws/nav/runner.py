"""MuJoCo integration: 10 Hz A* + MPPI control loop over the base-only nav scene.

MujocoEnv/MujocoSim cannot be reused here -- they hardcode arm slices
(pinch_site, gen3/base_link, cube qpos) that the nav scene does not have. Only
BaseController is reused, so the base still moves under the same Ruckig OTG
limits as the real robot.
"""

import argparse
import time
from dataclasses import dataclass, field, replace

import mujoco
import mujoco.viewer
import numpy as np

from constants import POLICY_CONTROL_PERIOD
from mujoco_env import BaseController
from nav import scene, viz
from nav.astar import astar, lookahead
from nav.cost_volume import CostVolumeConfig, DelayedCrowdSensor, build_cost_volume
from nav.grid import is_free, nearest_free, plan_grid, static_grid
from nav.metrics import Metrics
from nav.mppi import MPPI, MPPIConfig


class NavSim:
    """Base-only MuJoCo sim, stepped synchronously by the control loop."""

    def __init__(self, n_peds: int = 0, show_viewer: bool = True, real_time: bool = None):
        scene.write_scene(n_peds)
        self.model = mujoco.MjModel.from_xml_path(str(scene.SCENE_PATH))
        self.data = mujoco.MjData(self.model)
        self.model.body_gravcomp[:] = 1.0

        self.qpos = self.data.qpos[:3]
        self.qvel = self.data.qvel[:3]
        self.ctrl = self.data.ctrl[:3]
        self.base_controller = BaseController(self.qpos, self.qvel, self.ctrl, self.model.opt.timestep)

        self.n_substeps = round(POLICY_CONTROL_PERIOD / self.model.opt.timestep)
        # Redraw every ~16 ms of simulated time. Syncing once per control period
        # instead would show 10 fps: a whole 0.1 s of motion computed at once and
        # then a 90 ms wait, which reads as stutter even though the physics is smooth.
        self.sync_every = max(1, round(0.016 / self.model.opt.timestep))
        self.n_peds = n_peds
        self.real_time = show_viewer if real_time is None else real_time
        self.viewer = mujoco.viewer.launch_passive(
            self.model, self.data, show_left_ui=False, show_right_ui=False) if show_viewer else None
        self._next_wall_time = None

    def reset(self, pose):
        """pose: (3,) global (x, y, theta)."""
        mujoco.mj_resetData(self.model, self.data)
        self.base_controller.reset()          # zeros qpos and seeds the OTG
        self.qpos[:] = pose
        self.qvel[:] = 0.0
        self.ctrl[:] = self.qpos
        self.base_controller.otg_inp.current_position = self.qpos
        self.base_controller.otg_inp.current_velocity = self.qvel
        self.base_controller.otg_inp.target_position = self.qpos
        mujoco.mj_forward(self.model, self.data)
        self._next_wall_time = time.time()

    def step(self, base_pose):
        """One 10 Hz control period. base_pose: (3,) absolute global target."""
        command = {'base_pose': np.asarray(base_pose, float)}
        dt = self.model.opt.timestep
        for i in range(self.n_substeps):
            # The real pipeline's stale-command watchdog would freeze the base
            # whenever the sim runs off wall-clock speed; this loop always has a
            # fresh command, so keep it fed.
            self.base_controller.last_command_time = time.time()
            self.base_controller.control_callback(command)
            command = None
            mujoco.mj_step(self.model, self.data)

            if self.real_time:   # pace each substep, so the motion is drawn as it happens
                self._next_wall_time += dt
                time.sleep(max(0.0, self._next_wall_time - time.time()))
            if self.viewer is not None and (i + 1) % self.sync_every == 0:
                self.viewer.sync()

        if self.viewer is not None:
            self.viewer.sync()
        if not self.real_time:
            self._next_wall_time = time.time()

    def get_obs(self):
        return {'base_pose': self.qpos.copy(), 'base_vel': self.qvel.copy()}

    def set_ped_pos(self, xy):
        """xy: (n_peds, 2) world positions of the mocap pedestrians."""
        self.data.mocap_pos[:, :2] = xy

    @property
    def running(self):
        return self.viewer is None or self.viewer.is_running()

    def close(self):
        if self.viewer is not None:
            self.viewer.close()


@dataclass
class RunConfig:
    start: tuple[float, float, float] = (0.0, 0.0, 0.0)
    goal: tuple[float, float] = (8.0, 8.0)
    goal_theta: float = 0.0
    goal_tol: float = 0.15
    # Swept over the 6 acceptance pairs: 0.75 gives 6/6 with zero wall contacts,
    # 1.25-1.5 cuts the doorway corner (min clearance 0.25-0.30 m).
    lookahead_dist: float = 0.75
    # Velocity -> position-target lead time. The OTG plans to stop at the target,
    # so commanding x + u*dt caps the base at ~0.15 m/s. 1.5 s is the repo's own
    # idiom for this conversion (gamepad_teleop.py:71).
    cmd_lead: float = 1.5
    # A* costs up to ~130 ms on this grid, more than one control period, so it
    # must not run every step (PLAN section 9). The static map only needs a new
    # plan when the robot has been pushed off the old one.
    replan_dist: float = 1.0        # m off-path before replanning
    replan_every: int = 10          # control steps (1 Hz)
    max_time: float = 90.0          # s of sim time before giving up
    viz_samples: int = 24           # MPPI rollouts drawn in the viewer, -1 to draw nothing
    mppi: MPPIConfig = field(default_factory=MPPIConfig)


def navigate(sim: NavSim, cfg: RunConfig, grid=None, pgrid=None, crowd=None, cv_cfg=None, verbose=True):
    """Run one episode. Returns a Metrics summary dict.

    grid:  fine cost field for MPPI (res 0.05)
    pgrid: coarse grid for A* (res 0.10)
    """
    grid = grid if grid is not None else static_grid()
    pgrid = pgrid if pgrid is not None else plan_grid()
    mppi = MPPI(grid, cfg.mppi)
    goal = np.asarray(cfg.goal, float)
    if not is_free(pgrid, goal, scene.PLAN_CLEARANCE):
        goal = nearest_free(pgrid, goal, scene.PLAN_CLEARANCE)
        if verbose:
            print(f'goal {cfg.goal} is inside the inflation band, snapped to {goal.round(2)}')
    sim.reset(cfg.start)
    if crowd is not None:
        crowd.reset()
        sim.set_ped_pos(crowd.pos)

    sensor = DelayedCrowdSensor(cv_cfg, cfg.mppi.dt) if cv_cfg is not None else None
    m = Metrics(grid, dt=cfg.mppi.dt, goal_tol=cfg.goal_tol)
    # First plan happens before the timed loop starts.
    # ponytail: A* runs inline; move it to a worker thread only if replans get frequent.
    plan = lambda xy: astar(pgrid, xy if is_free(pgrid, xy, scene.PLAN_CLEARANCE)
                            else nearest_free(pgrid, xy, scene.PLAN_CLEARANCE),
                            goal, clearance=scene.PLAN_CLEARANCE)
    path = plan(np.asarray(cfg.start[:2], float))
    k, replans = 0, 0
    last_wall = time.time()

    while k * cfg.mppi.dt < cfg.max_time and sim.running:
        x = sim.get_obs()['base_pose']

        off_path = np.inf if path is None else np.linalg.norm(path - x[:2], axis=1).min()
        if path is None or off_path > cfg.replan_dist or (k and k % cfg.replan_every == 0):
            new_path = plan(x[:2])
            replans += 1
            if new_path is None and path is None:
                if verbose:
                    print('A*: no path to goal')
                break
            path = new_path if new_path is not None else path   # keep the last valid plan

        wp = lookahead(path, x[:2], cfg.lookahead_dist)

        t0 = time.perf_counter()
        cost_volume = None
        if crowd is not None and sensor is not None:
            p, v, age = sensor.observe(crowd.pos, crowd.vel)
            cost_volume = build_cost_volume(p, v, grid, cv_cfg, n_steps=cfg.mppi.T,
                                            dt=cfg.mppi.dt, age=age)
            assert cost_volume.shape[1:] == grid.occ.shape, 'cost volume grid mismatch'
        t1 = time.perf_counter()
        u = mppi.step(x, wp, cfg.goal_theta, cost_volume)
        solve_ms = 1e3 * (time.perf_counter() - t1)   # MPPI only, per the 20 ms budget
        cv_ms = 1e3 * (t1 - t0)

        if sim.viewer is not None and cfg.viz_samples >= 0:
            viz.draw(sim.viewer.user_scn, path=path, waypoint=wp, mppi=mppi,
                     n_samples=cfg.viz_samples)

        peds_now = None if crowd is None else crowd.pos.copy()   # pair with x, both pre-step
        sim.step(x + u * cfg.cmd_lead)
        if crowd is not None:
            crowd.step(cfg.mppi.dt, robot_xy=x[:2])
            sim.set_ped_pos(crowd.pos)

        now = time.time()
        m.update(x, u, goal, solve_ms, jitter_ms=1e3 * (now - last_wall - cfg.mppi.dt),
                 peds=peds_now, cv_ms=cv_ms)
        last_wall = now
        k += 1

        if np.linalg.norm(x[:2] - goal) < cfg.goal_tol:
            m.success = True
            break

        if verbose and k % 50 == 0:
            print(f'  t={k * cfg.mppi.dt:5.1f}s  x=({x[0]:5.2f},{x[1]:5.2f})  '
                  f'|goal|={np.linalg.norm(x[:2] - goal):4.2f}m  solve={solve_ms:4.1f}ms')

    return dict(m.summary(real_time=sim.real_time), replans=replans)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--start', type=float, nargs='+', default=None, metavar='X Y [THETA]')
    p.add_argument('--goal', type=float, nargs=2, default=None, metavar=('X', 'Y'))
    p.add_argument('--no-viewer', action='store_true')
    p.add_argument('--fast', action='store_true', help='run faster than real time (no viewer pacing)')
    p.add_argument('--peds', type=int, default=0, help='number of pedestrians (phase 2)')
    p.add_argument('--ped-policy', default='waypoint', choices=['constant', 'waypoint', 'sfm'])
    p.add_argument('--ped-sees-robot', action='store_true', help='pedestrians avoid the robot (reciprocity)')
    p.add_argument('--cost-volume', action='store_true', help='feed pedestrian predictions to MPPI (phase 3)')
    p.add_argument('--latency', type=float, default=0.0, help='cost volume age in seconds')
    p.add_argument('--no-replan', action='store_true',
                   help='plan once at the start and keep it (the green path stops moving)')
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--viz-samples', type=int, default=24,
                   help='MPPI rollouts drawn in the viewer (0 = plans only, -1 = nothing)')
    args = p.parse_args()

    # With a crowd, default to the episode that crosses the open crowd area.
    d_start, d_goal = ((scene.CROWD_START, scene.CROWD_GOAL) if args.peds
                       else ((0.0, 0.0, 0.0), (8.0, 8.0)))
    start = list(args.start) + [0.0] * (3 - len(args.start)) if args.start else list(d_start)
    cfg = RunConfig(start=tuple(start), goal=tuple(args.goal or d_goal),
                    viz_samples=args.viz_samples, mppi=MPPIConfig(seed=args.seed))
    if args.no_replan:
        # Both replan triggers disarmed; `path is None` still plans the first one.
        cfg = replace(cfg, replan_dist=np.inf, replan_every=1 << 30)
    sim = NavSim(n_peds=args.peds, show_viewer=not args.no_viewer, real_time=not args.fast)

    crowd, cv_cfg = None, None
    if args.peds:
        from nav.pedestrians import Crowd, CrowdConfig
        crowd = Crowd(CrowdConfig(n=args.peds, policy=args.ped_policy, sees_robot=args.ped_sees_robot,
                                  robot_start=tuple(start[:2]), seed=args.seed))
    if args.cost_volume:
        cv_cfg = CostVolumeConfig(latency=args.latency)

    try:
        out = navigate(sim, cfg, crowd=crowd, cv_cfg=cv_cfg)
    finally:
        sim.close()
    print({k: (round(v, 3) if isinstance(v, float) else v) for k, v in out.items()})


if __name__ == '__main__':
    main()
