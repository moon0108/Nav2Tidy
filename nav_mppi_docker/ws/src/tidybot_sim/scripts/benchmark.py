#!/usr/bin/env python3
"""nav/mppi.py vs nav2_mppi_controller over the four regions, to a CSV.

    python3 scripts/benchmark.py
    python3 scripts/benchmark.py --repeats 10 --controllers mppi
    python3 scripts/benchmark.py --scenarios utrap clutter --timeout 90

Gazebo is started once per controller, not once per episode: all four regions
live in one world and send_goal.teleport moves the robot between them.

Fairness. Every motion metric comes from this node sampling /odom at a fixed
20 Hz and feeding nav/metrics.py - the same Metrics the MuJoCo benchmark uses,
so the numbers are comparable with nav/results/ as well as between the two
controllers here. Nothing is taken from a planner's own report, because only one
of the two produces one.

Timing is asymmetric and the columns say so. nav2_mppi_controller publishes its
solve time nowhere, so the shared columns are what both sides expose - the
achieved cmd_vel period, and controller_server's own "missed its desired rate"
count - while the cm/mppi/pub breakdown is filled in for the challenger only.
A blank there means not measurable, not zero.
"""

import argparse
import csv
import glob
import os
import subprocess
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from nav import scene
from nav.grid import static_grid
from nav.metrics import Metrics
from send_goal import GOAL_THETA, teleport, wait_for_clock, yaw_of

PKG = Path(__file__).resolve().parent.parent
RESULTS = PKG / 'results'
# /odom sampling, and Metrics.dt with it. 10 Hz, not faster, for two reasons:
# metrics.py's own default is dt = 0.1 so these numbers line up with the MuJoCo
# runs in nav/results/, and grid.dist is quantised to 5 cm cells - at 0.5 m/s a
# 10 Hz sample steps exactly one cell, so a finer rate re-reads the same one
# while multiplying the third difference behind mean_jerk by 8.
SAMPLE_HZ = 10.0
SETTLE_S = 2.0                   # after a teleport, before the goal is judged
WZ_DEADBAND = 0.02               # rad/s; a sign change inside this is noise, not a flip
V_DISC = 0.5                     # m/s; the round translational limit both are judged against
CMD_HZ = 30.0                    # both controllers publish at this rate


def pct(v, f):
    return float(np.percentile(v, f)) if len(v) else float('nan')


class Bench(Node):

    def __init__(self):
        super().__init__('benchmark', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.odom = self.cmd = None
        self.costmap_seen = False
        self.cmd_t = []
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 50)
        self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self._on_costmap,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.grid = static_grid()

    def _on_odom(self, m):
        self.odom = m

    def _on_cmd(self, m):
        self.cmd = m
        self.cmd_t.append(self.get_clock().now().nanoseconds)

    def _on_costmap(self, _m):
        self.costmap_seen = True

    def pose(self):
        p = self.odom.pose.pose
        return np.array([p.position.x, p.position.y, yaw_of(p.orientation)])

    def twist(self):
        t = self.cmd or Twist()
        return np.array([t.linear.x, t.linear.y, t.angular.z])

    def wait_ready(self, timeout=90.0):
        """Stack is usable when the clock ticks, the BT answers, and both the
        costmap and odometry have arrived at least once."""
        wait_for_clock(self, timeout)
        assert self.client.wait_for_server(timeout_sec=timeout), 'no navigate_to_pose'
        end = time.time() + timeout
        while time.time() < end and not (self.costmap_seen and self.odom):
            rclpy.spin_once(self, timeout_sec=0.2)
        assert self.costmap_seen and self.odom, 'no costmap or odometry'

    def episode(self, name, timeout):
        (_, _, _), (gx, gy) = scene.SCENARIOS[name]
        teleport(name)
        t = self.get_clock().now()
        while (self.get_clock().now() - t).nanoseconds < SETTLE_S * 1e9:
            rclpy.spin_once(self, timeout_sec=0.05)

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(gx), float(gy)
        goal.pose.pose.orientation.z = np.sin(GOAL_THETA / 2)
        goal.pose.pose.orientation.w = np.cos(GOAL_THETA / 2)

        self.cmd_t.clear()
        m = Metrics(self.grid, dt=1.0 / SAMPLE_HZ, goal_tol=0.15,
                    robot_radius=scene.ROBOT_RADIUS)
        gxy = np.array([gx, gy])

        send = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if not handle.accepted:
            return None
        result = handle.get_result_async()

        t0 = self.get_clock().now()
        step = Duration(seconds=1.0 / SAMPLE_HZ)
        nxt = t0
        while not result.done():
            rclpy.spin_once(self, timeout_sec=0.01)
            now = self.get_clock().now()
            if now >= nxt:
                m.update(self.pose(), self.twist(), gxy)
                nxt = nxt + step
            if (now - t0).nanoseconds > timeout * 1e9:
                handle.cancel_goal_async()
                break
        while not result.done():
            rclpy.spin_once(self, timeout_sec=0.05)

        m.success = result.result().status == 4          # STATUS_SUCCEEDED
        out = m.summary(real_time=False)

        # Two numbers computed from the same 10 Hz cmd_vel samples for both
        # controllers. wz_flip_rate is the wobble itself: the fraction of
        # consecutive samples whose rotation reverses, ignoring near-zero wz.
        # vxy_over_frac says how much the constraint mismatch matters in practice
        # - nav2_mppi_controller can only express a per-axis box, so it may
        # command up to 0.707 m/s on a diagonal where the challenger is held to
        # the 0.5 m/s disc.
        U = np.array(m.controls).reshape(-1, 3)
        wz = U[:, 2]
        live = (np.abs(wz[1:]) > WZ_DEADBAND) & (np.abs(wz[:-1]) > WZ_DEADBAND)
        out['wz_flip_rate'] = round(float(np.mean((wz[1:] * wz[:-1] < 0) & live))
                                    if len(wz) > 1 else 0.0, 4)
        out['vxy_over_frac'] = round(float(np.mean(np.hypot(U[:, 0], U[:, 1]) > V_DISC + 1e-3))
                                     if len(U) else 0.0, 4)
        # Achieved control period, the one timing number both sides expose.
        d = np.diff(self.cmd_t) / 1e6 if len(self.cmd_t) > 2 else np.zeros(0)
        out['cmd_period_p50_ms'] = round(pct(d, 50), 3)
        out['cmd_period_p99_ms'] = round(pct(d, 99), 3)
        return out


def launch(controller, overrides=''):
    log = RESULTS / f'launch_{controller}.log'
    RESULTS.mkdir(exist_ok=True)
    f = log.open('w')
    cmd = ['ros2', 'launch', 'tidybot_sim', 'nav2.launch.py',
           f'controller:={controller}', 'headless:=true', 'rviz:=false']
    if overrides:
        # ros2 launch rejects a bare 'name:=' as malformed, so the default case
        # must omit the argument entirely rather than pass an empty one.
        cmd.append(f'mppi_overrides:={overrides}')
    p = subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT,
                         start_new_session=True)
    return p, log


def stop():
    subprocess.run([str(PKG / 'scripts' / 'stop.sh')], check=False)
    time.sleep(3)


def timing_of(new_files):
    """cm/mppi/pub percentiles from the CSV mppi_controller wrote for this run."""
    if not new_files:
        return {}
    rows = list(csv.DictReader(open(sorted(new_files, key=os.path.getmtime)[-1])))
    if not rows:
        return {}
    out = {}
    tot = []
    for k in ('cm_ms', 'mppi_ms', 'pub_ms'):
        v = [float(r[k]) for r in rows]
        out[f'{k[:-3]}_p50_ms'] = round(pct(v, 50), 3)
        tot.append(v)
    s = np.sum(tot, axis=0)
    out['solve_p50_ms'] = round(pct(s, 50), 3)
    out['solve_p99_ms'] = round(pct(s, 99), 3)
    out['eta_p50'] = round(pct([float(r['eta']) for r in rows], 50), 3)
    return out


COLUMNS = ['controller', 'variant', 'scenario', 'rep', 'success', 'nav_time', 'path_length',
           'min_wall_clearance', 'wall_collisions', 'mean_jerk', 'mean_speed',
           'freezing_time', 'wz_flip_rate', 'vxy_over_frac',
           'cmd_period_p50_ms', 'cmd_period_p99_ms', 'missed_rate',
           'solve_p50_ms', 'solve_p99_ms', 'cm_p50_ms', 'mppi_p50_ms', 'pub_p50_ms',
           'eta_p50']


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--controllers', nargs='+', default=['nav2', 'mppi'],
                    choices=['nav2', 'mppi'])
    ap.add_argument('--scenarios', nargs='+', default=sorted(scene.SCENARIOS),
                    choices=sorted(scene.SCENARIOS))
    ap.add_argument('--repeats', type=int, default=5)
    # One launch per entry, since MPPIConfig is read at node start.
    ap.add_argument('--variants', nargs='*', default=[''], metavar='OVERRIDES',
                    help='MPPIConfig overrides per run, e.g. "lam=2.0" "lam=10.0"')
    ap.add_argument('--timeout', type=float, default=120.0)
    ap.add_argument('--out', default=str(RESULTS / 'benchmark.csv'))
    a = ap.parse_args()

    rclpy.init()
    rows = []
    try:
        jobs = [(c, v) for c in a.controllers
                for v in (a.variants if c == 'mppi' else [''])]
        for controller, overrides in jobs:
            proc, log = launch(controller, overrides)
            node = Bench()                      # fresh per stack, so discovery is too
            try:
                node.wait_ready()
                for rep in range(a.repeats):
                    for name in a.scenarios:
                        before = set(glob.glob(str(RESULTS / 'timing_*.csv')))
                        off = log.stat().st_size
                        r = node.episode(name, a.timeout)
                        if r is None:
                            print(f'{controller}/{name}/{rep}: goal rejected')
                            continue
                        with log.open(errors='ignore') as f:
                            f.seek(off)
                            r['missed_rate'] = sum(
                                'missed its desired rate' in ln for ln in f)
                        if controller == 'mppi':
                            r.update(timing_of(
                                set(glob.glob(str(RESULTS / 'timing_*.csv'))) - before))
                        r.update(controller=controller, scenario=name, rep=rep,
                                 variant=overrides or 'default')
                        rows.append(r)
                        print(f"{controller:5s} {(overrides or 'default')[:14]:14s} {name:9s} rep{rep}  "
                              f"{'OK ' if r['success'] else 'FAIL'} "
                              f"t={r['nav_time']:5.1f}s len={r['path_length']:5.2f}m "
                              f"clear={r['min_wall_clearance']:.2f}m "
                              f"jerk={r['mean_jerk']:6.2f}")
            finally:
                node.destroy_node()
                stop()
                proc.wait(timeout=10)
    finally:
        rclpy.shutdown()

    if not rows:
        return 1
    Path(a.out).parent.mkdir(exist_ok=True)
    with open(a.out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=COLUMNS, extrasaction='ignore')
        w.writeheader()
        w.writerows(rows)
    print(f'\nwrote {a.out}')
    summarise(rows)
    return 0


def summarise(rows):
    def agg(sel):
        ok = [r for r in sel if r['success']]
        f = lambda k, src: np.mean([r[k] for r in src]) if src else float('nan')
        return (f'{np.mean([r["success"] for r in sel]):5.0%} '
                f'{f("nav_time", ok):7.1f} {f("path_length", ok):7.2f} '
                f'{f("min_wall_clearance", sel):8.2f} {f("mean_jerk", ok):8.2f} '
                f'{f("mean_speed", ok):6.3f} {f("wz_flip_rate", sel):6.1%} '
                f'{f("vxy_over_frac", sel):6.1%}')

    head = (f'{"controller/variant":20s} {"scenario":9s} {"succ":>5s} {"time":>7s} {"len":>7s} '
            f'{"clear":>8s} {"jerk":>8s} {"speed":>6s} {"wzflip":>6s} {">0.5":>6s}')
    print(f'\n{head}\n{"-" * len(head)}')
    for c in dict.fromkeys((r['controller'], r.get('variant', 'default')) for r in rows):
        sel = [r for r in rows
               if (r['controller'], r.get('variant', 'default')) == c]
        label = f'{c[0]}/{c[1]}'[:20]
        print(f'{label:20s} {"ALL":9s} {agg(sel)}')

    for v in dict.fromkeys(r.get('variant', 'default') for r in rows
                           if r['controller'] == 'mppi'):
        mp = [r for r in rows if r['controller'] == 'mppi'
              and r.get('variant', 'default') == v and 'solve_p50_ms' in r]
        if mp:
            g = lambda k: np.mean([r[k] for r in mp if k in r])
            print(f'\nmppi/{v} (nav2 exposes none of this): '
                  f'costmap {g("cm_p50_ms"):.2f} + mppi {g("mppi_p50_ms"):.2f} + '
                  f'publish {g("pub_p50_ms"):.2f} = p50 {g("solve_p50_ms"):.2f}, '
                  f'p99 {g("solve_p99_ms"):.2f} ms | eta p50 {g("eta_p50"):.2f} of K=512')


if __name__ == '__main__':
    raise SystemExit(main())
