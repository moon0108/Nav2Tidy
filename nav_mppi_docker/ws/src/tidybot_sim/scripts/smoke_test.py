#!/usr/bin/env python3
"""Stage 1 acceptance: does cmd_vel actually drive the base, on all three axes?

    ros2 launch tidybot_sim sim.launch.py headless:=true rviz:=false   # terminal 2
    python3 scripts/smoke_test.py                                      # terminal 3

VelocityControl applies the command in the MODEL frame, so the odom delta is
rotated back by the starting yaw before it is checked; the test therefore does
not care which scenario the robot was spawned into.
"""

import math
import sys

import rclpy
from geometry_msgs.msg import Twist
from nav import scene
from nav.grid import static_grid
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter

DUR = 2.0          # s of sim time per command
TOL = 0.30         # fraction of the expected displacement we insist on seeing
CASES = [
    ('forward ', (0.30, 0.0, 0.0)),
    ('strafe  ', (0.0, 0.30, 0.0)),
    ('rotate  ', (0.0, 0.0, 0.50)),
]


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


class Smoke(Node):
    def __init__(self):
        super().__init__('smoke_test', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.odom = None
        self.create_subscription(Odometry, '/odom',
                                 lambda m: setattr(self, 'odom', m), 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def wait_odom(self):
        self.odom = None
        while self.odom is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.odom

    def drive(self, v):
        """Hold v for DUR seconds of sim time; return the body-frame delta."""
        start = self.wait_odom()
        x0, y0 = start.pose.pose.position.x, start.pose.pose.position.y
        th0 = yaw_of(start.pose.pose.orientation)
        t0 = self.get_clock().now()

        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = v
        while (self.get_clock().now() - t0).nanoseconds < DUR * 1e9:
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)
        self.pub.publish(Twist())
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)

        end = self.wait_odom()
        dx = end.pose.pose.position.x - x0
        dy = end.pose.pose.position.y - y0
        c, s = math.cos(-th0), math.sin(-th0)
        dth = (yaw_of(end.pose.pose.orientation) - th0 + math.pi) % (2 * math.pi) - math.pi
        return c * dx - s * dy, s * dx + c * dy, dth


def main():
    rclpy.init()
    n = Smoke()
    o = n.wait_odom().pose.pose.position

    # The commands below are open-loop: from a tight spawn they drive straight
    # into a wall and the failure reads like a broken plugin instead of a bad
    # start. Same grid the planner uses, so the number means the same thing.
    grid = static_grid()
    need = scene.ROBOT_RADIUS + DUR * max(abs(c) for _, v in CASES for c in v[:2]) + 0.10
    col, row = grid.to_cell(o.x, o.y)
    have = grid.dist[row, col]
    assert have > need, (f'spawn ({o.x:.2f}, {o.y:.2f}) has {have:.2f} m clearance, '
                         f'needs {need:.2f}. Relaunch with scenario:=clutter.')
    print(f'  spawn ({o.x:.2f}, {o.y:.2f}), {have:.2f} m clear')
    bad = []
    for name, v in CASES:
        got = n.drive(v)
        want = tuple(c * DUR for c in v)
        ok = all(g > w * (1 - TOL) if w > 0 else abs(g) < 0.10
                 for g, w in zip(got, want))
        print(f'  {name} cmd={v}  body delta=({got[0]:+.3f}, {got[1]:+.3f}, {got[2]:+.3f})  '
              f'expected~({want[0]:+.2f}, {want[1]:+.2f}, {want[2]:+.2f})  {"OK" if ok else "FAIL"}')
        if not ok:
            bad.append(name.strip())
    n.destroy_node()
    rclpy.shutdown()
    print('all three axes drive' if not bad else f'FAILED: {bad}')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
