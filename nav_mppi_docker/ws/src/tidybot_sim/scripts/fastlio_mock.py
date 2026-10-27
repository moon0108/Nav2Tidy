#!/usr/bin/env python3
"""FAST-LIO stand-in, so team mode runs on the same TF chain as the real robot.

실기 구조(SLAM 팀 문서 2026-10-02)를 그대로 흉내 낸다.

    map -> camera_init -> body -> base_link
     |          |          |        `- static, 장착의 역 (이 노드)
     |          |          `- FAST-LIO, ~10 Hz        -> 이 노드
     |          `- transform_fusion, ~10 Hz (ICP 정합) -> 이 노드가 항등으로 대신
     `- Nav2 map. 실기에서는 FAST-LIO PCD 에서 만든다.

실기의 두 상단 구간이 **모두 동적**이라는 점이 중요하다. tf2 는 map->base_link 를
풀 때 두 구간의 공통 시각을 쓰므로, 한쪽만 동적이던 이전 구조와 지연 특성이 다르다.
그래서 이 노드가 map->camera_init 도 같은 틱에 같은 stamp 로 낸다 (dynamic_map_tf).

body 는 센서 내장 IMU 위치이지 베이스 중심이 아니다. 장착 변환은 body -> base_link
방향으로 낸다 -- body 가 이미 camera_init 의 자식이라 반대로 내면 base_link 에
부모가 둘 생긴다.

Input is the Gazebo ground-truth base pose off /odom. Only its pose is used; the
twist is dropped on purpose, exactly as FAST-LIO drops it.

    ros2 launch tidybot_sim nav2.launch.py controller:=team localization:=fastlio_mock
"""

import math

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter

# ── quaternion helpers, (x, y, z, w) ─────────────────────────────────────────
# tf_transformations 를 끌어오지 않는다. 필요한 연산이 셋뿐이고, 의존성 하나가
# 이미지에 있느냐 없느냐로 모드가 안 뜨는 것보다 스무 줄이 싸다.


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw,
                     aw * bw - ax * bx - ay * by - az * bz])


def qconj(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])


def qrot(q, v):
    """Rotate v by q. q must be unit."""
    t = 2.0 * np.cross(q[:3], v)
    return v + q[3] * t + np.cross(q[:3], t)


def rpy_to_q(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r / 2), math.sin(r / 2), math.cos(p / 2),
                              math.sin(p / 2), math.cos(y / 2), math.sin(y / 2))
    return np.array([sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
                     cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy])


class FastlioMock(Node):

    def __init__(self):
        super().__init__('fastlio_mock')
        # 기본값 없는 필수 파라미터. 장착 수치와 프레임 이름이 조용히 코드 기본값으로
        # 도는 일을 막는다 -- 실제로 sim.launch.py 가 엉뚱한 파일을 넘겨 그렇게 돌았고,
        # require_initialpose 테스트가 거짓으로 통과했다.
        required = [
            ('mount_xyz', Parameter.Type.DOUBLE_ARRAY),
            ('mount_rpy', Parameter.Type.DOUBLE_ARRAY),
            ('rate', Parameter.Type.DOUBLE),
            ('map_frame', Parameter.Type.STRING),
            ('camera_init_frame', Parameter.Type.STRING),
            ('imu_frame', Parameter.Type.STRING),
            ('base_frame', Parameter.Type.STRING),
            ('odom_topic', Parameter.Type.STRING),
            ('odometry_topic', Parameter.Type.STRING),
            ('initialpose_topic', Parameter.Type.STRING),
            ('dynamic_map_tf', Parameter.Type.BOOL),
            ('require_initialpose', Parameter.Type.BOOL),
        ]
        p = {q.name: q.value for q in self.declare_parameters('', required + [
            # base_link 기준 body 장착 pose. SLAM 팀 실측 (2026-10-02):
            # body -> base_link 가 (-0.321, -0.023, -0.401), 즉 그 역이 이 값이다.
            # 회전은 0. gen_world.py 의 정합성 assert 에는 넣지 않는다 -- 로봇이
            # 바뀌면 같이 바뀌는 값이지 두 세계가 합의해야 하는 값이 아니다.
            # 위치 소실 테스트. drop_after 초부터 drop_duration 초 동안 발행을 끊는다.
            # 0 이면 비활성. 시계는 첫 /odom 수신 시각(sim time)부터 돈다.
            ('drop_after', 0.0),
            ('drop_duration', 0.0),
            # map -> camera_init 을 10 Hz 동적 TF 로 낸다. 실기에서는
            # transform_fusion 이 ICP 로 계속 갱신하는 구간이고, 시뮬에서는 값이
            # 항등이지만 **동적이라는 사실 자체**가 TF 조회의 공통 시각을 바꾼다.
            # false 면 예전처럼 static 한 번 발행.
            # true 면 /initialpose 를 받기 전까지 map -> camera_init 을 내지 않는다.
            # 실기에서 정합이 초기화되기 전 상태를 재현한다. 그동안 map->base_link 는
            # 끊겨 있으므로, 그 상태에서 goal 을 보내면 LOCALIZATION_LOST 로 끝난다.
        ])}
        for n, _ in required:                 # 비어 있으면 여기서 죽는다
            self.get_parameter(n).value
        self.p = p
        self.q_bi = rpy_to_q(*[float(v) for v in p['mount_rpy']])
        self.p_bi = np.array([float(v) for v in p['mount_xyz']])

        self.src = None                 # 최신 /odom, 그대로 보관만 한다
        self.t0 = None                  # drop 시계의 원점 (sim time)
        self._drop_cfg = (float(p['drop_after']), float(p['drop_duration']))

        # require_initialpose 가 false 면 처음부터 정합된 것으로 본다.
        self.aligned = not bool(p['require_initialpose'])
        if not self.aligned:
            self.create_subscription(PoseWithCovarianceStamped, p['initialpose_topic'],
                                     self._on_initialpose, 10)

        self.tf_pub = tf2_ros.TransformBroadcaster(self)
        self.static_pub = tf2_ros.StaticTransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, p['odometry_topic'], 10)
        self.create_subscription(Odometry, p['odom_topic'], self._on_src, 10)
        self.create_timer(1.0 / float(p['rate']), self._tick)

        self._publish_mount()
        if not bool(p['dynamic_map_tf']):
            self._send(p['map_frame'], p['camera_init_frame'],
                       np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0]),
                       self.get_clock().now().to_msg(), static=True)
        self.get_logger().info(
            f"fastlio_mock: {p['camera_init_frame']} -> {p['imu_frame']} @ {p['rate']} Hz, "
            f"mount xyz={list(self.p_bi)} rpy={list(p['mount_rpy'])}"
            + (f", drop {p['drop_after']}..{p['drop_after'] + p['drop_duration']} s"
               if p['drop_duration'] > 0 else '')
            + (', map TF 동적' if p['dynamic_map_tf'] else ', map TF static')
            + ('' if self.aligned else ', /initialpose 대기 중'))

    def _send(self, parent, child, xyz, q, stamp, static=False):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id, t.child_frame_id = parent, child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
        (t.transform.rotation.x, t.transform.rotation.y,
         t.transform.rotation.z, t.transform.rotation.w) = q
        (self.static_pub if static else self.tf_pub).sendTransform(t)

    def _publish_mount(self):
        """static body -> base_link = 장착 변환의 역.

        body 가 이미 camera_init 의 자식이므로 이 방향이어야 트리가 선다.
        base_link -> body 로 내면 base_link 에 부모가 둘 생긴다.
        """
        q_ib = qconj(self.q_bi)
        self._send(self.p['imu_frame'], self.p['base_frame'],
                   -qrot(q_ib, self.p_bi), q_ib,
                   self.get_clock().now().to_msg(), static=True)

    def _on_initialpose(self, _msg):
        """정합 완료 신호로만 쓴다. pose 값은 보지 않는다 -- 시뮬의 camera_init 은
        이미 world 와 일치하므로 보정할 것이 없고, 재현하려는 것은 '초기화 전에는
        map->camera_init 이 없다'는 상태뿐이다."""
        if not self.aligned:
            self.aligned = True
            self.get_logger().info('/initialpose 수신: map -> camera_init 발행 시작')

    def _on_src(self, msg):
        self.src = msg                  # 최신값 보관만. 계산은 타이머에서.

    def _dropping(self, stamp_s):
        """주행 도중에 끊으려면 런타임에 파라미터를 바꾸는 수밖에 없다. 노드가 뜬
        시각부터 재는 고정 시계로는 goal 이 언제 시작될지 알 수 없기 때문이다.
        그래서 값이 바뀌면 그 순간을 새 원점으로 삼는다 - `ros2 param set` 한 번이
        곧 트리거다."""
        after = float(self.get_parameter('drop_after').value)
        dur = float(self.get_parameter('drop_duration').value)
        if (after, dur) != self._drop_cfg:
            self._drop_cfg, self.t0 = (after, dur), stamp_s
            if dur > 0:
                self.get_logger().warn(f'drop armed: {after}..{after + dur} s from now')
        if dur <= 0:
            return False
        if self.t0 is None:
            self.t0 = stamp_s
        el = stamp_s - self.t0
        return after <= el < after + dur

    def _tick(self):
        msg = self.src
        if msg is None:
            self.get_logger().warn(f"no {self.p['odom_topic']} yet",
                                   throttle_duration_sec=5.0)
            return
        stamp = msg.header.stamp       # 입력의 sim time 을 그대로 쓴다
        if self._dropping(stamp.sec + stamp.nanosec * 1e-9):
            self.get_logger().warn('dropping (localization loss test)',
                                   throttle_duration_sec=1.0)
            return

        # T_camera_init_imu = T_world_base o T_base_imu.
        # camera_init 은 Gazebo world 와 일치시킨다 (map -> camera_init 을 항등으로
        # 둘 수 있게). 실제 FAST-LIO 의 camera_init 은 켤 때의 IMU 자세라 중력
        # 정렬이 어긋나 기울어질 수 있다 -- ARCHITECTURE.md 의 "시뮬과 실제의 차이".
        pp, qq = msg.pose.pose.position, msg.pose.pose.orientation
        p_wb = np.array([pp.x, pp.y, pp.z])
        q_wb = np.array([qq.x, qq.y, qq.z, qq.w])
        q_wi = qmul(q_wb, self.q_bi)
        p_wi = p_wb + qrot(q_wb, self.p_bi)

        # 두 구간을 **같은 stamp 로 같은 틱에** 낸다. tf2 는 map->base_link 를 풀 때
        # 두 동적 구간의 공통 시각을 쓰므로, 엇갈려 내면 실기에 없는 지연이 생긴다.
        if self.aligned and self.p['dynamic_map_tf']:
            self._send(self.p['map_frame'], self.p['camera_init_frame'],
                       np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0]), stamp)
        elif not self.aligned:
            self.get_logger().warn('/initialpose 를 아직 못 받았다: map -> camera_init 보류',
                                   throttle_duration_sec=5.0)
        self._send(self.p['camera_init_frame'], self.p['imu_frame'], p_wi, q_wi, stamp)

        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = self.p['camera_init_frame']
        out.child_frame_id = self.p['imu_frame']
        (out.pose.pose.position.x, out.pose.pose.position.y,
         out.pose.pose.position.z) = p_wi
        (out.pose.pose.orientation.x, out.pose.pose.orientation.y,
         out.pose.pose.orientation.z, out.pose.pose.orientation.w) = q_wi
        # twist 는 비워 둔다. 실제 FAST-LIO 가 채우지 않으므로, 여기서 채우면
        # 시뮬에서만 동작하는 정지 판정을 만들게 된다.
        self.odom_pub.publish(out)


def main():
    rclpy.init()
    node = FastlioMock()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
