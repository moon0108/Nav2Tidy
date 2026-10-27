"""Dynamic obstacles: mocap-driven pedestrians with three motion policies.

`sees_robot` is deliberately a flag, not a default. Pedestrians that react to
the robot (reciprocity) make avoidance far easier, so results with and without
it are not comparable and must be reported separately.
"""

from dataclasses import dataclass

import numpy as np

from nav import scene


@dataclass
class CrowdConfig:
    n: int = 8
    policy: str = 'waypoint'        # 'constant' | 'waypoint' | 'sfm'
    # The base tops out at 0.5 m/s. Real 1.3 m/s walkers would simply run it down
    # -- a holonomic robot cannot sidestep something faster than itself often
    # enough. Matching the robot's speed preserves the robot:human speed ratio of
    # a real deployment (where both do about 1.3 m/s) instead of inverting it.
    speed: float = 0.5
    sees_robot: bool = False
    radius: float = scene.PED_RADIUS
    area: tuple = scene.CROWD_AREA
    min_spawn_dist: float = 1.0     # m between pedestrians, and from the robot start
    robot_start: tuple = scene.CROWD_START[:2]
    seed: int = 0
    # Social force parameters (Helbing & Molnar), used by policy='sfm'
    sfm_a: float = 2.0
    sfm_b: float = 0.3
    sfm_tau: float = 0.5


class Crowd:
    """pos: (n, 2), vel: (n, 2) world frame."""

    def __init__(self, cfg: CrowdConfig = None):
        self.cfg = cfg or CrowdConfig()
        assert self.cfg.policy in ('constant', 'waypoint', 'sfm'), self.cfg.policy
        self._assert_area_is_clear()
        self.rng = np.random.default_rng(self.cfg.seed)
        self.reset()

    def _assert_area_is_clear(self):
        """Pedestrians ignore walls, so their area must not contain any."""
        x0, y0, x1, y1 = self.cfg.area
        for w in scene.INTERIOR:
            overlap = not (w[2] <= x0 or w[0] >= x1 or w[3] <= y0 or w[1] >= y1)
            assert not overlap, f'crowd area {self.cfg.area} overlaps wall {w}'

    def _bounds(self):
        x0, y0, x1, y1 = self.cfg.area
        r = self.cfg.radius
        return np.array([x0 + r, y0 + r]), np.array([x1 - r, y1 - r])

    def _sample(self, k):
        # Inset by the radius: a goal on the border cannot be reached once the
        # position is clipped there, and the pedestrian stalls against the edge.
        lo, hi = self._bounds()
        return self.rng.uniform(lo, hi, size=(k, 2))

    def reset(self):
        cfg = self.cfg
        self.rng = np.random.default_rng(cfg.seed)   # idempotent: same seed, same crowd
        pts = []
        blocked = [np.asarray(cfg.robot_start, float)]
        while len(pts) < cfg.n:                       # rejection sampling, n is small
            p = self._sample(1)[0]
            if all(np.linalg.norm(p - q) > cfg.min_spawn_dist for q in blocked + pts):
                pts.append(p)
        self.pos = np.array(pts)
        self.goals = self._sample(cfg.n)              # waypoint / sfm target
        self.home = self.pos.copy()                   # waypoint policy returns here
        theta = self.rng.uniform(0, 2 * np.pi, cfg.n)
        self.vel = cfg.speed * np.stack([np.cos(theta), np.sin(theta)], axis=1)
        if cfg.policy != 'constant':
            self.vel = self._towards(self.goals)

    def _towards(self, targets):
        d = targets - self.pos
        n = np.linalg.norm(d, axis=1, keepdims=True)
        return self.cfg.speed * d / np.maximum(n, 1e-6)

    def step(self, dt, robot_xy=None):
        cfg = self.cfg
        if cfg.policy != 'constant':
            # Ping-pong between goal and spawn, so nobody parks on their target.
            reached = np.linalg.norm(self.goals - self.pos, axis=1) < 0.3
            if reached.any():
                self.goals[reached], self.home[reached] = self.home[reached], self.goals[reached].copy()

        if cfg.policy == 'waypoint':
            self.vel = self._towards(self.goals)
        elif cfg.policy == 'sfm':
            self.vel = self._sfm(dt, robot_xy)         # 'constant' holds its velocity

        self.pos = self.pos + self.vel * dt
        self._bounce()

    def _sfm(self, dt, robot_xy):
        cfg = self.cfg
        f = (self._towards(self.goals) - self.vel) / cfg.sfm_tau      # drive to desired velocity

        d = self.pos[:, None, :] - self.pos[None, :, :]               # (n, n, 2)
        dist = np.linalg.norm(d, axis=2)
        np.fill_diagonal(dist, np.inf)
        mag = cfg.sfm_a * np.exp((2 * cfg.radius - dist) / cfg.sfm_b)
        f += (mag[..., None] * d / dist[..., None]).sum(axis=1)

        if cfg.sees_robot and robot_xy is not None:
            dr = self.pos - np.asarray(robot_xy, float)
            nr = np.maximum(np.linalg.norm(dr, axis=1, keepdims=True), 1e-6)
            f += cfg.sfm_a * np.exp((cfg.radius + scene.ROBOT_RADIUS - nr) / cfg.sfm_b) * dr / nr

        v = self.vel + f * dt
        speed = np.linalg.norm(v, axis=1, keepdims=True)
        return v * np.minimum(1.0, cfg.speed / np.maximum(speed, 1e-6))

    def _bounce(self):
        """Reflect off the area border. Keeps the crowd where it was declared to
        be -- pedestrians have no wall model of their own."""
        lo, hi = self._bounds()
        out = (self.pos < lo) | (self.pos > hi)
        self.vel = np.where(out, -self.vel, self.vel)
        self.goals = np.where(out, np.clip(2 * self.pos - self.goals, lo, hi), self.goals)
        self.pos = np.clip(self.pos, lo, hi)
