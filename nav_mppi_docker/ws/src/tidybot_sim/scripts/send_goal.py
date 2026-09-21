#!/usr/bin/env python3
"""Send one NavigateToPose goal for a named scenario and report the outcome.

    python3 scripts/send_goal.py doorway [timeout_s]
    python3 scripts/send_goal.py all [timeout_s]      # every scenario -> results CSV

This is the stage 2 acceptance check in script form - the same thing a "2D Goal
Pose" click in RViz does, but repeatable and with a number at the end. Stage 4's
benchmark runner grows from it.
"""

import csv
import math
import subprocess
import sys
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter

from nav import scene

GOAL_THETA = 0.0       # RunConfig.goal_theta; SmacPlanner2D emits no orientation


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def teleport(name, world='tidy'):
    """Drop the robot at the scenario's start pose without restarting Gazebo.

    All four regions share one world precisely so this is possible; ign transport
    has no rclpy binding, so the CLI is the short way in. Odometry is the model
    pose, so it follows the jump and needs no reset.
    """
    (x, y, yaw), _ = scene.SCENARIOS[name]
    req = (f'name: "tidybot", position {{x: {x}, y: {y}, z: 0.01}}, '
           f'orientation {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}')
    r = subprocess.run(['ign', 'service', '-s', f'/world/{world}/set_pose',
                        '--reqtype', 'ignition.msgs.Pose',
                        '--reptype', 'ignition.msgs.Boolean',
                        '--timeout', '3000', '--req', req],
                       capture_output=True, text=True)
    assert 'true' in r.stdout.lower(), f'set_pose failed: {r.stdout.strip()} {r.stderr.strip()}'


def wait_for_clock(node, timeout=30.0):
    """Block until /clock has been seen.

    With use_sim_time the clock reads 0 until the first /clock message lands, so
    a t0 taken too early makes the very first elapsed-time check fire at the full
    sim uptime and the timeout trips instantly.
    """
    import time as _t
    deadline = _t.time() + timeout
    while node.get_clock().now().nanoseconds == 0 and _t.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    assert node.get_clock().now().nanoseconds, 'no /clock; is the sim running?'


class Goal(Node):
    def __init__(self):
        super().__init__('send_goal', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.rows = []

    def send(self, name, timeout, move=True):
        (sx, sy, _), (gx, gy) = scene.SCENARIOS[name]
        if not self.client.wait_for_server(timeout_sec=20.0):
            print('navigate_to_pose action server never appeared')
            return 1

        wait_for_clock(self)
        if move:
            teleport(name)
            # Let the costmaps and the TF buffer catch up with the jump before a
            # goal is judged against them.
            t = self.get_clock().now()
            while (self.get_clock().now() - t).nanoseconds < 2e9:
                rclpy.spin_once(self, timeout_sec=0.1)

        msg = NavigateToPose.Goal()
        msg.pose.header.frame_id = 'map'
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(gx), float(gy)
        msg.pose.pose.orientation.z = math.sin(GOAL_THETA / 2)
        msg.pose.pose.orientation.w = math.cos(GOAL_THETA / 2)

        t0 = self.get_clock().now()
        send = self.client.send_goal_async(msg)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if not handle.accepted:
            print(f'{name}: goal rejected')
            return 1

        print(f'{name}: ({sx:.2f}, {sy:.2f}) -> ({gx:.2f}, {gy:.2f}), waiting up to {timeout:.0f} s')
        result = handle.get_result_async()
        # Sim clock: a stalled Gazebo must not read as a fast failure.
        while not result.done():
            rclpy.spin_once(self, timeout_sec=0.2)
            if (self.get_clock().now() - t0).nanoseconds > timeout * 1e9:
                handle.cancel_goal_async()
                print(f'{name}: TIMEOUT after {timeout:.0f} s of sim time')
                return 1

        dt = (self.get_clock().now() - t0).nanoseconds / 1e9
        code = result.result().status
        ok = code == 4                                   # STATUS_SUCCEEDED
        print(f'{name}: {"REACHED" if ok else f"FAILED (status {code})"} in {dt:.1f} s')
        self.rows.append({'scenario': name, 'reached': int(ok), 'status': code,
                          'time_s': round(dt, 2), 'goal_x': gx, 'goal_y': gy})
        return 0 if ok else 1


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else 'corridor'
    names = sorted(scene.SCENARIOS) if name == 'all' else [name]
    if any(k not in scene.SCENARIOS for k in names):
        print(f'unknown scenario {name!r}; pick one of {sorted(scene.SCENARIOS)} or "all"')
        return 2
    timeout = float(sys.argv[2]) if len(sys.argv) > 2 else 120.0

    rclpy.init()
    n = Goal()
    rc = max(n.send(k, timeout) for k in names)
    if name == 'all':
        # Which planner is answering follow_path decides the filename. Reading
        # the node list beats a flag: a mislabelled run is worse than no run,
        # and stage 4 compares these two files directly.
        label = 'mppi' if 'mppi_controller' in n.get_node_names() else 'nav2'
        out = Path(__file__).resolve().parent.parent / 'results' / f'run_{label}.csv'
        out.parent.mkdir(exist_ok=True)
        with out.open('w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(n.rows[0]))
            w.writeheader()
            w.writerows(n.rows)
        print(f'wrote {out}')
    n.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
