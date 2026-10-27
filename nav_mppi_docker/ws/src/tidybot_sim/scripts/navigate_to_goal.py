#!/usr/bin/env python3
"""NavigateToGoal action server for the team-integration stack.

The frontend, and nothing else. Every decision about how the robot moves lives
in executor.py, which does not know this action type exists; this file is the
only place that mentions athome_interfaces. Swap the interface package for the
team's original and the msg boundary block below is all there is to reconcile.

    ros2 launch tidybot_sim nav2.launch.py controller:=team scenario:=clutter

Rejection vs abort, because ROS 2 makes this choice for you: a rejected goal
carries no Result at all, so a reason code cannot reach the client. Therefore

    실행 중 두 번째 goal  -> REJECT      (reason 목록에 busy 가 없다)
    map_version 불일치    -> 수락 후 즉시 abort, reason = MAP_MISMATCH

Only the second one has a reason to deliver, so only the second one is accepted
just to be failed.
"""

import threading
import time

import numpy as np
import rclpy
from athome_interfaces.action import NavigateToGoal
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Header

import executor as ex
from executor import Executor, load_robot_config
# Config parsing and a quaternion helper, imported rather than copied.
# mppi_controller.py is frozen (its benchmark numbers must keep meaning what
# they meant), and neither function touches its behaviour. If it is ever
# refactored this breaks loudly at import, not quietly at runtime.
from mppi_controller import override, yaw_of
from nav.mppi import MPPI, MPPIConfig

# ── msg 경계 ─────────────────────────────────────────────────────────────────
# 임시 athome_interfaces 를 팀 원문으로 교체할 때 고칠 곳은 이 블록뿐이다.

_R = NavigateToGoal.Result

# executor 의 문자열 outcome -> (status, reason). 'map_mismatch' 는 executor 가
# 모르는, 이 파일에서만 나는 실패다.
OUTCOME_TO_RESULT = {
    'reached':           (_R.STATUS_SUCCEEDED, _R.REASON_NONE),
    'canceled':          (_R.STATUS_CANCELED,  _R.REASON_NONE),
    'blocked':           (_R.STATUS_FAILED,    _R.REASON_BLOCKED),
    'no_progress':       (_R.STATUS_FAILED,    _R.REASON_NO_PROGRESS),
    'timeout':           (_R.STATUS_FAILED,    _R.REASON_TIMEOUT),
    'localization_lost': (_R.STATUS_FAILED,    _R.REASON_LOCALIZATION_LOST),
    'internal':          (_R.STATUS_FAILED,    _R.REASON_INTERNAL),
    'map_mismatch':      (_R.STATUS_FAILED,    _R.REASON_MAP_MISMATCH),
}
# executor 에 outcome 이 하나 늘면 여기서 import 시각에 터진다. 매핑을 빠뜨린 채
# KeyError 로 result 를 못 채우는 것보다 낫다.
assert set(OUTCOME_TO_RESULT) == set(ex.OUTCOMES) | {'map_mismatch'}, \
    f'unmapped outcomes: {set(ex.OUTCOMES) - set(OUTCOME_TO_RESULT)}'

# executor 의 소문자 phase -> 스펙의 대문자. 'idle' 은 goal 을 수락했지만 첫 solve 가
# 아직 phase 를 정하지 않은 창이라 ACCEPTED 로 나간다.
PHASE_OUT = {'idle': 'ACCEPTED', 'driving': 'DRIVING',
             'rotating': 'ROTATING', 'stopping': 'STOPPING'}
assert set(PHASE_OUT) == set(ex.PHASES)


def _pose_stamped(pose, frame, stamp):
    """(x, y, theta) or None -> PoseStamped. None 은 원점 항등으로 나간다."""
    m = PoseStamped()
    m.header.frame_id, m.header.stamp = frame, stamp
    if pose is None:
        m.pose.orientation.w = 1.0
        return m
    m.pose.position.x, m.pose.position.y = float(pose[0]), float(pose[1])
    m.pose.orientation.z = float(np.sin(pose[2] / 2))
    m.pose.orientation.w = float(np.cos(pose[2] / 2))
    return m


def goal_from_msg(msg, robot, global_frame):
    """Goal -> Executor.start() 인자. 프레임이 다르면 ValueError.

    경로가 비어 있으면 목표점 한 점으로 읽는다 = 제자리 회전. 탐색 시스템이 관측
    회전을 요청하는 형태(path 한 점, yaw 만 다름)와 같은 취급이다.
    """
    for what, frame in (('goal', msg.goal.header.frame_id),
                        ('path', msg.path.header.frame_id)):
        if frame and frame != global_frame:
            raise ValueError(f'{what} is in {frame!r}, this node works in {global_frame!r}')
    goal_xy = np.array([msg.goal.pose.position.x, msg.goal.pose.position.y])
    path_xy = np.array([[p.pose.position.x, p.pose.position.y]
                        for p in msg.path.poses], float).reshape(-1, 2)
    return dict(
        path_xy=path_xy if len(path_xy) else goal_xy.reshape(1, 2),
        goal_xy=goal_xy,
        goal_yaw=yaw_of(msg.goal.pose.orientation),
        # 0 은 "노드 기본값을 써라"는 뜻이다 (스펙). 음수도 같게 취급한다.
        xy_tol=msg.xy_tolerance if msg.xy_tolerance > 0 else robot['xy_tolerance'],
        yaw_tol=msg.yaw_tolerance if msg.yaw_tolerance > 0 else robot['yaw_tolerance'],
        # 0 이면 executor 가 경로 형태를 보고 drive/rotate 기본값을 고른다. 판정을
        # 두 곳에 두지 않기 위해 여기서는 그대로 통과시킨다.
        timeout=msg.timeout_sec)


def result_msg(outcome, pose, stopped, detail, frame, stamp):
    r = NavigateToGoal.Result()
    r.status, r.reason = OUTCOME_TO_RESULT[outcome]
    r.final_pose = _pose_stamped(pose, frame, stamp)
    r.stopped = bool(stopped)
    r.detail = detail
    return r


def feedback_msg(pose, remaining, phase, frame, stamp):
    f = NavigateToGoal.Feedback()
    f.current_pose = _pose_stamped(pose, frame, stamp)
    f.remaining_distance = float(remaining)
    f.phase = PHASE_OUT[phase]
    return f

# ── 노드 ─────────────────────────────────────────────────────────────────────


class NavigateToGoalNode(Node):

    def __init__(self):
        super().__init__('navigate_to_goal')
        # 기본값 없는 것들: 액션/토픽 이름, 감시 임계, 로봇 설정 경로. 파일이 안
        # 들어오면 여기서 죽는다 (executor.read_params 참조).
        required = [
            ('action_name', Parameter.Type.STRING),
            ('heartbeat_topic', Parameter.Type.STRING),
            # 2 s 로 끊김을 판정한다. 0 이면 감시를 끈다 - heartbeat 를 보내지 않는
            # `ros2 action send_goal` 로 손으로 찔러볼 때만 쓴다.
            ('heartbeat_timeout', Parameter.Type.DOUBLE),
            ('robot_config', Parameter.Type.STRING),
        ]
        optional = [
            ('feedback_frequency', 10.0),       # 스펙 요구는 5 Hz 이상
            # gen_world.py 가 maps/tidy.version 에 쓴 값을 launch 가 넣어준다.
            # 비어 있으면 검사를 건너뛴다.
            ('map_version', ''),
            ('mppi_overrides', ''),
        ]
        self.p = ex.read_params(self, required, optional)
        self.robot = load_robot_config(self.p['robot_config'])

        cfg = override(MPPIConfig(), str(self.p['mppi_overrides']))
        if self.p['mppi_overrides']:
            self.get_logger().warn(f"MPPIConfig overridden: {self.p['mppi_overrides']}")
        assert abs(cfg.dt - 1.0 / self.p['solve_frequency']) < 1e-9, (
            f"solve_frequency {self.p['solve_frequency']} != 1/MPPIConfig.dt "
            f'{1 / cfg.dt}; step() shifts the warm start by exactly dt per call')

        # One optimizer for the life of the node: the warm start is most of what
        # makes K = 512 enough, and a fresh MPPI per goal would throw it away.
        self.mppi = MPPI(None, cfg)
        self.ex = Executor(self, self.mppi, self.p, self.robot)

        # 액션과 heartbeat 는 executor 의 solve/publish 타이머와 다른 group 에 둔다.
        # 같은 group 이면 100 ms 짜리 solve 가 cancel 과 heartbeat 를 막아, 정지
        # 명령이 한 주기 늦게 나간다.
        self.api = ReentrantCallbackGroup()
        self.busy_lock, self.busy = threading.Lock(), False
        self.hb_t = None

        self.create_subscription(Header, self.p['heartbeat_topic'], self._on_heartbeat,
                                 10, callback_group=self.api)
        self.server = ActionServer(
            self, NavigateToGoal, self.p['action_name'],
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=self.api)
        self.get_logger().info(
            f"{self.p['action_name']} up: K={cfg.K}, T={cfg.T}, dt={cfg.dt}, "
            f"solve {self.p['solve_frequency']} Hz, publish {self.p['publish_frequency']} Hz, "
            f"feedback {self.p['feedback_frequency']} Hz, "
            f"map_version {self.p['map_version'] or '(unchecked)'}")
        if self.p['heartbeat_timeout'] <= 0:
            self.get_logger().warn(
                'heartbeat_timeout = 0: 송신 노드가 죽어도 감지하지 못한다. '
                '수동 테스트용 설정이며 팀 연동에서는 쓰면 안 된다')

    # ── 액션 ─────────────────────────────────────────────────────────────────

    def _on_goal(self, _req):
        """실행 중이면 REJECT. 수락과 start() 사이의 창은 busy 플래그가 막는다."""
        with self.busy_lock:
            if self.busy or self.ex.state != ex.IDLE:
                self.get_logger().warn('goal rejected: another goal is running')
                return GoalResponse.REJECT
            self.busy = True
            return GoalResponse.ACCEPT

    def _execute(self, handle):
        """순서 계약: 정지 판정 -> executor IDLE -> busy 해제 -> result 전송.

        rclpy 는 이 콜백이 반환한 뒤에 result 를 보내고, finally 는 그 반환보다
        먼저 돈다. executor 쪽은 _done 과 IDLE 을 한 락 안에서 함께 바꾼다. 따라서
        result 를 받은 클라이언트가 곧바로 다음 goal 을 보내도 busy 로 거절되지
        않는다 -- 관측 회전 4연발이 이 순서에 걸려 있다.
        """
        t0 = time.monotonic()
        try:
            return self._run(handle, t0)
        finally:
            with self.busy_lock:
                self.busy = False

    def _run(self, handle, t0):
        req = handle.request
        frame = self.p['global_frame']
        rid = req.request_id or '(no id)'

        try:
            g = goal_from_msg(req, self.robot, frame)
        except ValueError as e:
            return self._fail_now(handle, 'internal', str(e))

        want = str(self.p['map_version'])
        if want and req.map_version != want:
            # 수락한 뒤 abort 하는 유일한 경우. REJECT 면 reason 이 전달되지 않는다.
            return self._fail_now(handle, 'map_mismatch',
                                  f'expected {want!r}, goal carries {req.map_version!r}')

        # 한 점 경로는 제자리 회전이라는 뜻이다. 회전의 내용은 yaw 하나뿐이므로 위치는
        # **메시지에서 읽지 않고 수락 시점의 실제 pose 로 덮어쓴다**. 그것이 hold 기준이
        # 된다: d_goal ~ 0 이라 첫 tick 에 yaw 판정으로 넘어가고, 회전 중 표류하면
        # _carrot 이 이 점을 waypoint 로 주어 제자리를 지킨다.
        #
        # 예전에는 |pose - goal_xy| > xy_tol 이면 internal 로 거절했다. 그런데 탐색
        # 시스템이 goal.position 을 어느 프레임의 '현재 위치'로 채우는지는 우리가 통제할
        # 수 없다 -- /localization 은 body 프레임이고 (README 의 child_frame_id: body),
        # LiDAR 가 전면 돌출이라 body <-> base_link 오프셋이 xy_tolerance 0.15 m 를 넘을
        # 수 있다. 그러면 **모든 회전 goal 이 거절된다.** 거절해서 얻는 것이 없다: 위치를
        # 쓰지 않으므로 틀린 위치가 로봇을 움직이게 하지도 않는다.
        # 어긋남 자체는 d 로 로그에 남긴다 -- 0.15 m 를 넘으면 프레임이 다르다는 증거다.
        if len(g['path_xy']) <= 1:
            pose, _, _ = self.ex.feedback()
            if pose is None:
                # hold 기준점을 만들 수 없다. 이것만은 여전히 거절한다.
                return self._fail_now(handle, 'internal',
                                      'no TF; cannot anchor a single-pose (rotation) goal')
            d = float(np.linalg.norm(pose[:2] - g['goal_xy']))
            self.get_logger().info(
                f'rotation goal: holding at ({pose[0]:.2f}, {pose[1]:.2f}), '
                f"goal yaw {g['goal_yaw']:+.3f} rad, yaw_tol {g['yaw_tol']:.4f} rad; "
                f'message xy was {d:.2f} m away (ignored)')
            g['goal_xy'] = pose[:2].copy()
            g['path_xy'] = pose[:2].reshape(1, 2).copy()

        if not self.ex.start(**g):
            return self._fail_now(handle, 'internal', 'executor was not idle')
        # heartbeat 시계는 goal 수락 시점부터 돈다: 한 번도 안 오는 것과 도중에
        # 끊기는 것은 감시 대상으로서 같은 사고다.
        self.hb_t = self.get_clock().now()
        if self.p['heartbeat_timeout'] <= 0:
            self.get_logger().warn(f'goal {rid}: heartbeat 감시가 꺼져 있다 '
                                   '(heartbeat_timeout = 0)')
        self.get_logger().info(f'goal {rid}: {len(g["path_xy"])} poses -> '
                               f'({g["goal_xy"][0]:.2f}, {g["goal_xy"][1]:.2f}, '
                               f'{g["goal_yaw"]:+.2f})')

        detail, canceling = '', False
        rate = self.create_rate(self.p['feedback_frequency'])
        try:
            while rclpy.ok():
                if handle.is_cancel_requested and not canceling:
                    detail, canceling = 'cancel requested by the client', True
                    self.ex.cancel()
                elif not canceling:
                    age = self._heartbeat_age()
                    if age is not None:
                        detail, canceling = f'heartbeat lost (age={age:.1f}s)', True
                        self.ex.fail('internal')

                out = self.ex.take_outcome()
                if out is not None:
                    break
                pose, rem, phase = self.ex.feedback()
                handle.publish_feedback(
                    feedback_msg(pose, rem, phase, frame, self.get_clock().now().to_msg()))
                rate.sleep()
            else:
                return self._fail_now(handle, 'internal', 'shutting down')
        finally:
            self.destroy_rate(rate)

        outcome, pose, stopped, ex_detail = out
        dt = time.monotonic() - t0
        # 우선순위: 프론트엔드가 판단한 것(cancel, heartbeat) > executor 가 판단한 것
        # (입력 상실, stale) > 일반 문구.
        detail = detail or ex_detail or f'{outcome} in {dt:.1f}s'
        self.get_logger().info(f'goal {rid}: {outcome}, stopped={stopped}, {dt:.1f}s')
        {'reached': handle.succeed, 'canceled': handle.canceled}.get(outcome, handle.abort)()
        return result_msg(outcome, pose, stopped, detail, frame,
                          self.get_clock().now().to_msg())

    def _fail_now(self, handle, outcome, detail):
        """Fail before the robot ever moved: stopped is true because it is."""
        self.get_logger().warn(f'{outcome}: {detail}')
        handle.abort()
        pose, _, _ = self.ex.feedback()
        return result_msg(outcome, pose, True, detail, self.p['global_frame'],
                          self.get_clock().now().to_msg())

    # ── heartbeat ────────────────────────────────────────────────────────────

    def _on_heartbeat(self, _msg):
        """수신 시각을 노드 clock 으로 찍는다. 메시지의 header.stamp 는 쓰지 않는다 -
        감시 대상이 '송신 노드가 살아 있는가'이지 '지연이 얼마인가'가 아니다."""
        self.hb_t = self.get_clock().now()

    def _heartbeat_age(self):
        """끊긴 경우 마지막 수신 이후 경과 초, 아니면 None.

        bool 이 아니라 나이를 돌려주는 것은 result 의 detail 에 숫자를 남기기
        위해서다 - "heartbeat lost" 만으로는 2 s 만에 끊긴 것인지 30 s 동안
        안 온 것인지 사후에 구분할 수 없다.
        """
        to = self.p['heartbeat_timeout']
        if to <= 0 or self.hb_t is None:
            return None
        age = (self.get_clock().now() - self.hb_t).nanoseconds / 1e9
        return age if age > to else None


def main():
    rclpy.init()
    node = NavigateToGoalNode()
    # solve 타이머, publish 타이머, 액션 execute, cancel, heartbeat 가 동시에
    # 돌아야 한다. 단일 스레드면 execute 의 대기 루프가 solve 를 굶긴다.
    exec_ = MultiThreadedExecutor()
    exec_.add_node(node)
    try:
        exec_.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
