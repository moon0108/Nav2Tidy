#!/usr/bin/env python3
"""FollowPath action server driven by nav/mppi.py, in place of controller_server.

    ros2 launch tidybot_sim nav2.launch.py controller:=mppi scenario:=utrap

nav/mppi.py is not touched: no change to the update rule, the cost terms or the
weights. Everything here is interface - frames in, a Grid-shaped object in,
(vx, vy, wz) out.

controller_server still runs, with its follow_path action remapped away: it owns
local_costmap, published_footprint and clear_entirely_local_costmap, which
behavior_server and the recovery branch of the BT both depend on. With no goal
ever reaching it, its own MPPI never computes anything.

Frames. Three conversions, all explicit even where the transform is identity
today, so attaching AMCL later changes nothing here:

    robot pose   TF odom->base_link, fresh every solve      -> MPPI x0
    global path  Path header (map) -> costmap frame (odom)  -> lookahead()
    costmap      already in odom; the frame is asserted, not assumed
    cmd_vel      MPPI works in odom, cmd_vel is base_link   -> rotate by -theta

The last one is the quiet one. nav/mppi.py's docstring says "Action u = (vx, vy,
omega) global frame"; Gazebo's VelocityControl and Nav2 both read cmd_vel in the
body frame. Rotating at publish time rather than at solve time matters while the
base is turning: the command is held between solves.

Rate. step() shifts the warm start by exactly one horizon step, dt = 0.1 s, per
call. Calling it at 30 Hz would advance the plan 100 ms per 33 ms of real time
and burn the horizon three times too fast, and fixing that means editing step().
So the solve runs at 1/dt = 10 Hz - the rate the real base is controlled at
(constants.py POLICY_CONTROL_FREQ) - and cmd_vel is published at 30 Hz holding
the latest command, which is what the brief asks for and what VelocityControl
does with it anyway.
"""

import csv
import math
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Twist
from nav2_msgs.action import FollowPath
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from nav.astar import lookahead
from nav.mppi import MPPI, MPPIConfig

RESULTS = Path(__file__).resolve().parent.parent / 'results'


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


class CostmapGrid:
    """The three members of nav/grid.py's Grid that mppi.py actually reads.

    Nothing is baked: the costmap arrives already inflated by Nav2's
    inflation_layer, which is the point of handing inflation to Nav2. The cost
    field's shape is therefore Nav2's exponential, not Grid.bake()'s - same
    range, different curve.

    Scale. Getting this wrong is silent and fatal, so spelled out:

        Grid.bake()         d <= robot_radius        -> 10.0
                            else exp(-(d - r)/decay) -> (0, 1]
        OccupancyGrid       LETHAL 254               -> 100
                            INSCRIBED 253            -> 99
                            inflated 1..252          -> 1..98
                            NO_INFORMATION 255       -> -1

    So the inflated band tops out at 98, not at 10. Dividing the whole range by
    10 makes that band ten times hotter than the one w_obs = 20.0 was tuned
    against, and at 200 per step it buries the goal term: measured, the robot
    parked at the mouth of a 1.05 m pillar gap for 120 s, oscillating, never
    entering. 99 and 100 fold to bake()'s lethal 10.0; everything below scales
    by 98 so the inflated band lands back in (0, 1].
    """

    LETHAL = 10.0
    INSCRIBED = 99          # OccupancyGrid value of costmap_2d's 253

    def __init__(self, msg: OccupancyGrid, expect_frame):
        assert msg.header.frame_id == expect_frame, (
            f'costmap is in {msg.header.frame_id!r}, expected {expect_frame!r}; '
            'the rollouts and the cost lookup would be in different frames')
        info = msg.info
        self.origin = np.array([info.origin.position.x, info.origin.position.y])
        self.res = info.resolution
        raw = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
        # -1 is NO_INFORMATION. local_costmap leaves track_unknown_space at
        # false, so it does not occur; if it ever does, free is the matching
        # reading (that is the costmap's own default_value) and the global path
        # has already vetted the topology anyway.
        self.cost = np.where(raw < 0, 0.0,
                             np.where(raw >= self.INSCRIBED, self.LETHAL,
                                      raw / (self.INSCRIBED - 1)))

    def lookup(self, pts):
        """pts: (..., 2) world xy in the costmap frame -> (...,) cost."""
        idx = np.floor((pts - self.origin) / self.res).astype(np.int32)
        h, w = self.cost.shape
        c, r = idx[..., 0], idx[..., 1]
        inside = (c >= 0) & (c < w) & (r >= 0) & (r < h)
        # Grid.lookup clamps, which is right for a static map covering the whole
        # arena. This one is a 6 m rolling window: clamping would report the
        # edge cell for everything past it, so a rollout could leave the
        # observed box and look cheap. The 2.5 s horizon reaches 1.25 m and the
        # window gives 3 m, so this should never fire - it is a guard, not a
        # behaviour.
        return np.where(inside,
                        self.cost[np.clip(r, 0, h - 1), np.clip(c, 0, w - 1)],
                        self.LETHAL)


class MppiController(Node):

    def __init__(self):
        super().__init__('mppi_controller')
        p = self.declare_parameters('', [
            ('solve_frequency', 10.0),
            ('publish_frequency', 30.0),
            ('K', 512),
            ('lookahead_dist', 0.75),
            ('goal_switch_dist', 0.75),
            ('xy_goal_tolerance', 0.15),
            ('yaw_goal_tolerance', 0.25),
            ('progress_radius', 0.5),
            ('progress_time', 10.0),
            ('viz_samples', 24),
            ('costmap_topic', '/local_costmap/costmap'),
            ('costmap_frame', 'odom'),
            ('base_frame', 'base_link'),
            ('log_timing', True),
        ])
        self.p = {q.name: q.value for q in p}

        self.cfg = MPPIConfig(K=int(self.p['K']))
        assert abs(self.cfg.dt - 1.0 / self.p['solve_frequency']) < 1e-9, (
            f"solve_frequency {self.p['solve_frequency']} != 1/MPPIConfig.dt "
            f'{1 / self.cfg.dt}; step() shifts the warm start by exactly dt per call')

        group = ReentrantCallbackGroup()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.costmap = None
        self.create_subscription(
            OccupancyGrid, self.p['costmap_topic'], self._on_costmap,
            # Matches nav2_costmap_2d's publisher: QoS(1) transient_local.
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=group)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.viz_pub = self.create_publisher(MarkerArray, '~/rollouts', 1)

        self.u_odom = np.zeros(3)          # latest command, costmap frame
        self.driving = False
        self.lock = threading.Lock()

        # The default BT recomputes the path at 1 Hz and re-sends FollowPath
        # each time, so goals arrive about once a second for one navigation.
        # nav2_util::SimpleActionServer preempts: newest path wins, the running
        # loop stops. Accepting them all instead - which is what a plain rclpy
        # ActionServer on a reentrant group does - leaves a dozen control loops
        # alive at once, fighting over cmd_vel and over the CPU.
        self.preempt = threading.Event()
        self.exec_lock = threading.Lock()
        # One optimizer across the whole navigation: a fresh MPPI per goal would
        # throw the warm start away every second, which is most of what makes
        # K=512 enough.
        self.mppi = MPPI(None, self.cfg)
        self.rows = []
        self.nav_t0 = None
        self.create_timer(1.0 / self.p['publish_frequency'], self._publish,
                          callback_group=group)

        self.server = ActionServer(
            self, FollowPath, 'follow_path',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=group)
        self.get_logger().info(
            f'follow_path up: K={self.cfg.K}, T={self.cfg.T}, dt={self.cfg.dt}, '
            f"solve {self.p['solve_frequency']} Hz, publish {self.p['publish_frequency']} Hz")

    def _on_goal(self, _goal):
        self.preempt.set()          # ask whatever is running to unwind
        return GoalResponse.ACCEPT

    # ---------------------------------------------------------------- frames

    def _on_costmap(self, msg):
        self.costmap = msg

    def _pose(self, frame):
        """(x, y, theta) of base_frame in `frame`, from TF."""
        t = self.tf_buffer.lookup_transform(
            frame, self.p['base_frame'], rclpy.time.Time(),
            timeout=Duration(seconds=0.2)).transform
        return np.array([t.translation.x, t.translation.y, yaw_of(t.rotation)])

    def _path_to(self, msg, frame):
        """Path poses as (N, 2) in `frame`.

        The lookup runs even when the path is already in `frame` - map->odom is
        a static identity here, but reading it explicitly is what lets AMCL be
        dropped in without touching this node. One rigid transform for the whole
        array beats 90-odd per-pose tf2 calls at every solve.
        """
        src = msg.header.frame_id or 'map'
        xy = np.array([[q.pose.position.x, q.pose.position.y] for q in msg.poses])
        t = self.tf_buffer.lookup_transform(
            frame, src, rclpy.time.Time(), timeout=Duration(seconds=2.0)).transform
        th = yaw_of(t.rotation)
        self.get_logger().info(
            f'path: {len(xy)} poses {src} -> {frame} via '
            f'({t.translation.x:+.3f}, {t.translation.y:+.3f}, {th:+.3f} rad)')
        c, s = math.cos(th), math.sin(th)
        return xy @ np.array([[c, s], [-s, c]]) + [t.translation.x, t.translation.y]

    def _publish(self):
        """30 Hz. Rotates the held command into the base frame with a fresh
        heading: between solves the base keeps turning, and a command rotated
        once at solve time would steer into the turn."""
        msg = Twist()
        if self.driving:
            with self.lock:
                u = self.u_odom.copy()
            try:
                th = self._pose(self.p['costmap_frame'])[2]
            except tf2_ros.TransformException:
                return
            c, s = math.cos(-th), math.sin(-th)
            msg.linear.x = float(c * u[0] - s * u[1])
            msg.linear.y = float(s * u[0] + c * u[1])
            msg.angular.z = float(u[2])
        self.cmd_pub.publish(msg)

    # ---------------------------------------------------------------- action

    def _execute(self, handle):
        with self.exec_lock:        # the previous execution has unwound by here
            return self._follow(handle)

    def _follow(self, handle):
        self.preempt.clear()
        frame = self.p['costmap_frame']
        result = FollowPath.Result()

        try:
            path = self._path_to(handle.request.path, frame)
        except tf2_ros.TransformException as e:
            self.get_logger().error(f'cannot transform path into {frame}: {e}')
            handle.abort()
            return result
        if len(path) < 2:
            self.get_logger().error('path has fewer than 2 poses')
            handle.abort()
            return result

        goal_xy = path[-1]
        goal_th = yaw_of(handle.request.path.poses[-1].pose.orientation)
        mppi = self.mppi                     # grid is swapped in per solve
        if not self.rows:                    # first goal of this navigation
            mppi.reset()
            self.nav_t0 = self.get_clock().now()
        rate = self.create_rate(self.p['solve_frequency'])

        t_start = self.nav_t0
        last_move, last_move_t = None, t_start
        self.driving = True
        outcome = 'aborted'

        while rclpy.ok():
            if handle.is_cancel_requested:
                outcome = 'canceled'
                break
            if self.preempt.is_set():
                outcome = 'preempted'       # a newer path arrived; that goal drives now
                break

            t0 = time.perf_counter()
            msg = self.costmap
            if msg is None:
                self.get_logger().warn('no costmap yet', throttle_duration_sec=2.0)
                rate.sleep()
                continue
            try:
                mppi.grid = CostmapGrid(msg, frame)
                x = self._pose(frame)
            except tf2_ros.TransformException as e:
                self.get_logger().warn(f'TF: {e}', throttle_duration_sec=2.0)
                rate.sleep()
                continue
            t1 = time.perf_counter()

            d_goal = float(np.linalg.norm(x[:2] - goal_xy))
            if d_goal < self.p['xy_goal_tolerance']:
                dth = abs((x[2] - goal_th + math.pi) % (2 * math.pi) - math.pi)
                if dth < self.p['yaw_goal_tolerance']:
                    outcome = 'reached'
                    break

            # Near the end the carrot becomes the goal itself: the path's last
            # pose sits inside SmacPlanner2D's own 0.25 m tolerance, so chasing
            # it would park the robot short of where the goal checker measures.
            wp = goal_xy if d_goal < self.p['goal_switch_dist'] \
                else lookahead(path, x[:2], dist=self.p['lookahead_dist'])
            u = mppi.step(x, wp, goal_theta=goal_th, cost_volume=None)  # N_o = 0
            t2 = time.perf_counter()

            with self.lock:
                self.u_odom = u
            self._draw(mppi, frame)
            fb = FollowPath.Feedback()
            fb.distance_to_goal = d_goal
            fb.speed = float(np.hypot(u[0], u[1]))
            handle.publish_feedback(fb)
            t3 = time.perf_counter()

            now = self.get_clock().now()
            if last_move is None or np.linalg.norm(x[:2] - last_move) > self.p['progress_radius']:
                last_move, last_move_t = x[:2].copy(), now
            elif (now - last_move_t).nanoseconds > self.p['progress_time'] * 1e9:
                self.get_logger().error(
                    f"no {self.p['progress_radius']} m of progress in "
                    f"{self.p['progress_time']} s")
                break

            self.rows.append({
                't_sim': round((now - self.nav_t0).nanoseconds / 1e9, 4),
                'cm_ms': round(1e3 * (t1 - t0), 3),
                'mppi_ms': round(1e3 * (t2 - t1), 3),
                'pub_ms': round(1e3 * (t3 - t2), 3),
                'x': round(x[0], 4), 'y': round(x[1], 4), 'theta': round(x[2], 4),
                'vx': round(u[0], 4), 'vy': round(u[1], 4), 'wz': round(u[2], 4),
                'dist_to_goal': round(d_goal, 4),
            })
            rate.sleep()

        self.destroy_rate(rate)
        if outcome != 'preempted':
            # Only a terminal outcome stops the robot. Zeroing on preemption
            # would put a stutter into every replan.
            self.driving = False
            with self.lock:
                self.u_odom = np.zeros(3)
            self.cmd_pub.publish(Twist())
            self._report(self.rows, outcome,
                         (self.get_clock().now() - t_start).nanoseconds / 1e9)
            self.rows = []

        {'reached': handle.succeed, 'canceled': handle.canceled}.get(
            outcome, handle.abort)()
        return result

    # ------------------------------------------------------------ viz + logs

    def _draw(self, mppi, frame):
        n = int(self.p['viz_samples'])
        if n <= 0 or not hasattr(mppi, 'last_X'):
            return
        stamp = self.get_clock().now().to_msg()

        def mk(i, kind, width, rgba):
            m = Marker()
            m.header.frame_id, m.header.stamp = frame, stamp
            m.ns, m.id, m.type, m.action = 'mppi', i, kind, Marker.ADD
            m.scale.x = width
            m.pose.orientation.w = 1.0
            (m.color.r, m.color.g, m.color.b, m.color.a) = rgba
            return m

        # One LINE_LIST for every sample beats n markers through the bus. A
        # uniform slice, not the top-weighted ones: at lam = 0.5 the softmax
        # puts ~all the weight on one rollout, so "top n" would draw one line
        # n times (viz.py makes the same choice).
        samples = mk(0, Marker.LINE_LIST, 0.004, (0.2, 0.6, 1.0, 0.25))
        for k in range(0, mppi.K, max(1, mppi.K // n)):
            xy = mppi.last_X[k, :, :2]
            for a, b in zip(xy, xy[1:]):
                samples.points.append(_pt(a))
                samples.points.append(_pt(b))
        best = mk(1, Marker.LINE_STRIP, 0.02, (1.0, 0.85, 0.1, 0.95))
        best.points.extend(_pt(q) for q in mppi.last_nominal[:, :2])
        self.viz_pub.publish(MarkerArray(markers=[samples, best]))

    def _report(self, rows, outcome, dt):
        if not rows:
            self.get_logger().warn(f'{outcome} with no control cycles')
            return
        q = lambda key, f: sorted(r[key] for r in rows)[int(f * (len(rows) - 1))]
        self.get_logger().info(
            f'{outcome} in {dt:.1f} s, {len(rows)} cycles | '
            f"costmap p50 {q('cm_ms', .5):.2f} p99 {q('cm_ms', .99):.2f} | "
            f"mppi p50 {q('mppi_ms', .5):.2f} p99 {q('mppi_ms', .99):.2f} | "
            f"publish p50 {q('pub_ms', .5):.2f} p99 {q('pub_ms', .99):.2f} ms "
            f"(budget {1e3 / self.p['solve_frequency']:.0f} ms)")
        if not self.p['log_timing']:
            return
        RESULTS.mkdir(exist_ok=True)
        out = RESULTS / f'timing_{int(time.time())}_{outcome}.csv'
        with out.open('w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
        self.get_logger().info(f'wrote {out}')


def _pt(xy):
    from geometry_msgs.msg import Point
    p = Point()
    p.x, p.y = float(xy[0]), float(xy[1])
    return p


def main():
    rclpy.init()
    node = MppiController()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
