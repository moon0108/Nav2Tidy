#!/usr/bin/env python3
"""Stand-in for the team's exploration system, for testing controller:=team.

Does what the real client will do - ask Nav2's planner for an A* path, hand it
to /athome/navigate_to_goal, and keep a 2 Hz heartbeat alive - plus the four
ways of breaking it that the contract has to survive.

    python3 scripts/mock_explorer.py --scenario clutter
    python3 scripts/mock_explorer.py --scenario clutter --rotate4
    python3 scripts/mock_explorer.py 8.9 8.9 --cancel-after 4
    python3 scripts/mock_explorer.py 8.9 8.9 --drop-heartbeat 3
    python3 scripts/mock_explorer.py 8.9 8.9 --bad-map-version
    python3 scripts/mock_explorer.py 8.9 8.9 --double-goal
    python3 scripts/mock_explorer.py --raw-path 8.3 4.2 8.3 1.0    # into a wall

navigate_to_goal.py is deliberately NOT imported: that would drag in the
executor and the optimiser, and a test tool that contains half its subject is
not a test. The status/reason names come from the .action's own constants.
"""

import argparse
import math
import subprocess
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from athome_interfaces.action import NavigateToGoal
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Path as PathMsg
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import tf2_ros
from std_msgs.msg import Header

# nav.scene 과 send_goal 은 **시뮬 전용**이다 (Gazebo 텔레포트, /clock 대기).
# 실기에는 /clock 이 없어서 wait_for_clock 이 영원히 멈춘다. 그래서 최상단에서
# import 하지 않고 --scenario 분기 안에서만 불러온다.

_R = NavigateToGoal.Result
STATUS_NAME = {getattr(_R, n): n[7:] for n in dir(_R) if n.startswith('STATUS_')}
REASON_NAME = {getattr(_R, n): n[7:] for n in dir(_R) if n.startswith('REASON_')}


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


class MockExplorer(Node):

    def __init__(self):
        super().__init__('mock_explorer')
        group = ReentrantCallbackGroup()
        self.n = 0

        # heartbeat 는 노드가 뜨자마자 돈다. navigate_to_goal 의 감시 시계는 goal
        # 수락 시점부터 도니, goal 보다 늦게 시작하면 모든 goal 이 실패한다.
        self.hb_on = True
        self.hb_pub = self.create_publisher(Header, '/athome/executor/heartbeat', 10)
        self.create_timer(0.5, self._heartbeat, callback_group=group)

        self.planner = ActionClient(self, ComputePathToPose, '/compute_path_to_pose',
                                    callback_group=group)
        self.nav = ActionClient(self, NavigateToGoal, '/athome/navigate_to_goal',
                                callback_group=group)
        self.clear = self.create_client(ClearEntireCostmap,
                                        '/global_costmap/clear_entirely_global_costmap',
                                        callback_group=group)

        # /goal_pose 수신용. --listen-goal-pose 에서만 쓴다.
        self.pending = None                 # 가장 최근에 받은 목표 (PoseStamped)
        self.new_goal = threading.Event()
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer, self)

        share = get_package_share_directory('tidybot_sim')
        self.map_version = (Path(share) / 'maps' / 'tidy.version').read_text().strip() \
            if (Path(share) / 'maps' / 'tidy.version').exists() else ''
        self.last_phase = None

    def _heartbeat(self):
        if self.hb_on:
            self.hb_pub.publish(Header(stamp=self.get_clock().now().to_msg(),
                                       frame_id='mock_explorer'))

    def clear_costmap(self):
        """텔레포트 직후 global costmap 을 비운다.

        순간이동하면 odom->base_link TF 가 튀고, 그 사이에 들어온 스캔이 어긋난
        pose 로 변환돼 유령 장애물을 칠한다. 세션 내내 쌓이면 실제로 길이 막혀서
        planner 가 빈 경로로 ABORT 한다 (측정됨). 텔레포트가 그 마크를 무효화하는
        행위이므로 여기서 지우는 것이 맞다.

        주의: 이것은 검증 도구 쪽 처방일 뿐이다. team 모드 자체에는 costmap 을
        지우는 주체가 없다 - BT 를 들어내면서 그 복구 분기가 같이 사라졌다.
        실제 운용에서 누가 지울지는 팀과 정해야 한다 (ARCHITECTURE.md 14 절).
        """
        if not self.clear.wait_for_service(timeout_sec=5.0):
            print('경고: clear_entirely_global_costmap 서비스를 찾지 못했다')
            return
        self._wait(self.clear.call_async(ClearEntireCostmap.Request()), timeout=10.0)

    def _pose(self, x, y, yaw=0.0):
        m = PoseStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y = float(x), float(y)
        m.pose.orientation.z = math.sin(yaw / 2)
        m.pose.orientation.w = math.cos(yaw / 2)
        return m

    @staticmethod
    def _wait(fut, timeout=60.0):
        """spin 은 다른 스레드가 돌린다. 여기서는 완료만 기다린다."""
        t0 = time.time()
        while not fut.done():
            if time.time() - t0 > timeout:
                raise TimeoutError('future never completed')
            time.sleep(0.02)
        return fut.result()

    # ── 경로 ─────────────────────────────────────────────────────────────────

    def plan(self, xy, yaw=0.0):
        """Nav2 planner 로 A* 경로를 받는다. 실제 탐색 시스템이 하는 첫 단계."""
        assert self.planner.wait_for_server(timeout_sec=15), '/compute_path_to_pose 없음'
        req = ComputePathToPose.Goal()
        req.goal, req.use_start = self._pose(*xy, yaw), False
        gh = self._wait(self.planner.send_goal_async(req))
        assert gh.accepted, 'planner rejected the goal'
        path = self._wait(gh.get_result_async()).result.path
        assert len(path.poses), 'planner returned an empty path'
        return path

    def _on_goal_pose(self, msg):
        """RViz 2D Goal Pose. 최신 것만 들고, 처리는 메인 루프가 한다."""
        if msg.header.frame_id and msg.header.frame_id != 'map':
            print(f'무시: /goal_pose frame 이 {msg.header.frame_id!r} 이다 (map 이어야 한다)')
            return
        self.pending = msg
        self.new_goal.set()
        print(f'\n/goal_pose 수신 ({msg.pose.position.x:+.2f}, {msg.pose.position.y:+.2f}, '
              f'{yaw_of(msg.pose.orientation):+.2f})')

    def current_xy(self, timeout=5.0):
        """TF map -> base_link. 실기에서 로봇이 지금 어디 있는지."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            try:
                t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                return np.array([t.transform.translation.x, t.transform.translation.y])
            except Exception:
                time.sleep(0.1)
        raise TimeoutError('TF map -> base_link 를 못 읽었다')

    def straight_path(self, goal_xy, step=0.05):
        """현재 위치 -> 목표 직선. planner 가 없을 때 쓴다.

        장애물을 전혀 보지 않는다. 경로가 벽을 관통하면 MPPI 가 회피하다
        BLOCKED 나 NO_PROGRESS 로 끝난다 -- 테스트용이라는 뜻이다.
        """
        a = self.current_xy()
        d = np.linalg.norm(goal_xy - a)
        n = max(2, int(d / step) + 1)
        return self.raw_path(np.linspace(a, goal_xy, n))

    def raw_path(self, pts):
        """planner 를 건너뛴 임의 경로. BLOCKED / NO_PROGRESS 를 유발하는 유일한 길이고,
        탐색 시스템이 낡은 지도로 만든 경로를 보내는 상황이기도 하다."""
        p = PathMsg()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.poses = [self._pose(x, y) for x, y in pts]
        return p

    def hold_path(self, xy):
        """제자리 회전용 한 점 경로."""
        return self.raw_path([xy])

    # ── 요청 ─────────────────────────────────────────────────────────────────

    def _on_feedback(self, msg):
        f = msg.feedback
        if f.phase != self.last_phase:
            self.last_phase = f.phase
            print(f'      phase -> {f.phase}  (remaining {f.remaining_distance:.2f} m)')

    @staticmethod
    def drop_localization(dur):
        """fastlio_mock 의 발행을 dur 초 동안 끊는다.

        파라미터를 바꾸는 순간이 곧 시계의 원점이므로, drop_after 0 은 '지금 당장'
        이라는 뜻이 된다. rclpy 에 동기 파라미터 클라이언트가 없어 CLI 를 쓴다 -
        검증 도구에서 한 번 부르는 것이라 그 편이 짧다.
        """
        subprocess.run(['ros2', 'param', 'set', '/fastlio_mock', 'drop_after', '0.0'],
                       capture_output=True)
        subprocess.run(['ros2', 'param', 'set', '/fastlio_mock', 'drop_duration', str(dur)],
                       capture_output=True)

    def navigate(self, path, goal_xy, yaw, label, cancel_after=None, drop_hb=None,
                 bad_version=False, double=False, drop_loc=None, yield_to_new=False):
        self.n += 1
        self.last_phase = None
        g = NavigateToGoal.Goal()
        g.request_id = f'mock-{self.n}'
        g.goal = self._pose(*goal_xy, yaw)
        g.path = path
        g.map_version = 'deadbeef' if bad_version else self.map_version
        # 0 = "노드 기본값을 써라". demo.yaml 이 실제로 읽히는지 확인하는 경로이기도 하다.
        g.xy_tolerance = g.yaw_tolerance = g.timeout_sec = 0.0

        assert self.nav.wait_for_server(timeout_sec=15), '/athome/navigate_to_goal 없음'
        head = f'#{self.n} {label:<8} ({goal_xy[0]:.2f}, {goal_xy[1]:.2f}, {yaw:+.2f})' \
               f'  {len(path.poses)} pose{"s" if len(path.poses) != 1 else ""}'
        print(head)

        t0 = time.time()
        gh = self._wait(self.nav.send_goal_async(g, feedback_callback=self._on_feedback))
        if not gh.accepted:
            print(f'      -> REJECTED (Result 없음 - 설계대로)   {time.time() - t0:.1f}s\n')
            return None

        rf, sent_double, canceled, dropped = gh.get_result_async(), False, False, False
        dropped_loc = False
        while not rf.done():
            el = time.time() - t0
            if double and not sent_double and el > 0.5:
                sent_double = True
                self._second_goal(path, goal_xy, yaw)
            if cancel_after and not canceled and el > cancel_after:
                canceled = True
                print(f'      [{el:.1f}s] cancel 요청')
                gh.cancel_goal_async()
            if yield_to_new and self.new_goal.is_set() and not canceled:
                # 새 목표가 왔다. 기존 goal 을 취소하고, **result 를 받은 뒤에**
                # 새 goal 을 보낸다 (executor 의 순서 계약이 그것을 보장한다).
                canceled = True
                print(f'      [{el:.1f}s] 새 목표 수신 -> 기존 goal cancel')
                gh.cancel_goal_async()
            if drop_loc and not dropped_loc and el > drop_loc[0]:
                dropped_loc = True
                print(f'      [{el:.1f}s] 위치 발행 {drop_loc[1]}s 중단')
                self.drop_localization(drop_loc[1])
            if drop_hb and not dropped and el > drop_hb:
                dropped = True
                self.hb_on = False
                print(f'      [{el:.1f}s] heartbeat 중단')
            if el > 300:
                raise TimeoutError('result never arrived')
            time.sleep(0.05)

        res = rf.result().result
        dt = time.time() - t0
        fp = res.final_pose.pose
        err_xy = math.hypot(fp.position.x - goal_xy[0], fp.position.y - goal_xy[1])
        err_th = abs(wrap(yaw_of(fp.orientation) - yaw))
        print(f'      -> {STATUS_NAME.get(res.status, res.status)}/'
              f'{REASON_NAME.get(res.reason, res.reason)}  stopped={str(res.stopped).lower()}'
              f'  {dt:.1f}s  err=({err_xy:.3f} m, {err_th:.3f} rad)')
        print(f'         detail: {res.detail}\n')
        self.hb_on = True                       # 다음 요청을 위해 되살린다
        return res

    def _second_goal(self, path, goal_xy, yaw):
        """실행 중 두 번째 goal. REJECT 가 기대 동작이다."""
        g = NavigateToGoal.Goal()
        g.request_id = 'mock-double'
        g.goal, g.path, g.map_version = self._pose(*goal_xy, yaw), path, self.map_version
        gh = self._wait(self.nav.send_goal_async(g))
        print(f'      [double-goal] {"ACCEPTED (버그!)" if gh.accepted else "REJECTED (정상)"}')


def listen_loop(node, straight):
    """RViz 2D Goal Pose 로 들어오는 목표를 계속 처리한다."""
    node.create_subscription(PoseStamped, '/goal_pose', node._on_goal_pose, 10)
    print('/goal_pose 대기 중. RViz 의 "2D Goal Pose" 로 찍으세요.  (Ctrl-C 로 종료)')
    if not straight:
        print('경로는 /compute_path_to_pose 로 만듭니다.')
    while rclpy.ok():
        node.new_goal.wait(timeout=0.5)
        if not node.new_goal.is_set():
            continue
        node.new_goal.clear()
        msg = node.pending
        goal_xy = np.array([msg.pose.position.x, msg.pose.position.y])
        yaw = yaw_of(msg.pose.orientation)
        try:
            path = node.straight_path(goal_xy) if straight else node.plan(goal_xy, yaw)
        except AssertionError as e:
            print(f'  경로 계획 실패: {e}  -> goal 을 보내지 않는다')
            continue
        except Exception as e:
            print(f'  경로 계획 실패: {type(e).__name__}: {e}  -> goal 을 보내지 않는다')
            continue
        try:
            node.navigate(path, goal_xy, yaw, 'straight' if straight else 'drive',
                          yield_to_new=True)
        except AssertionError as e:
            # 액션 서버가 아직 없다거나 하는 경우. 이 모드는 RViz 를 켜 두고 오래
            # 쓰는 것이라, 한 번 실패했다고 리스너가 죽으면 안 된다.
            print(f'  전송 실패: {e}  -> 계속 대기')
        except Exception as e:
            print(f'  전송 실패: {type(e).__name__}: {e}  -> 계속 대기')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('xy', nargs='*', type=float, metavar='X Y [YAW]',
                    help='목표 지점 (map frame)')
    ap.add_argument('--scenario',
                    help='시작점으로 텔레포트하고 그 시나리오의 목표를 쓴다 (시뮬 전용)')
    ap.add_argument('--rotate4', action='store_true',
                    help='도착 후 90도 4방향 순차 관측 회전')
    ap.add_argument('--raw-path', nargs='+', type=float, metavar='X Y',
                    help='planner 를 건너뛴 임의 경로 (BLOCKED / NO_PROGRESS 유발용)')
    ap.add_argument('--cancel-after', type=float, metavar='SEC')
    ap.add_argument('--drop-heartbeat', type=float, metavar='SEC')
    ap.add_argument('--drop-localization', nargs=2, type=float, metavar=('AFTER', 'DUR'),
                    help='주행 AFTER 초 뒤부터 DUR 초 동안 fastlio_mock 발행 중단')
    ap.add_argument('--bad-map-version', action='store_true')
    ap.add_argument('--double-goal', action='store_true')
    ap.add_argument('--listen-goal-pose', action='store_true',
                    help='RViz 2D Goal Pose(/goal_pose)를 받아 계속 주행한다 (실기용)')
    ap.add_argument('--straight-path', action='store_true',
                    help='planner 없이 현재 위치->목표 직선 경로를 쓴다 (테스트용)')
    ap.add_argument('--map-version', default='',
                    help="goal 에 실을 map_version. 빈 값이면 노드가 검사를 건너뛴다")
    a = ap.parse_args()

    rclpy.init()
    node = MockExplorer()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    try:
        if a.map_version:
            node.map_version = a.map_version
        elif a.listen_goal_pose:
            # 실기에서는 시뮬 맵의 판본(maps/tidy.version)을 보내면 안 된다.
            # 빈 값이면 노드가 검사를 건너뛴다 -- 팀과 형식이 미합의라 그게 맞다.
            node.map_version = ''
        if a.listen_goal_pose:
            # 실기 경로. /clock 도 Gazebo 도 없다.
            if node.map_version:
                print(f'map_version {node.map_version}')
            listen_loop(node, a.straight_path)
            return

        from send_goal import wait_for_clock       # 시뮬 전용 (/clock 대기)
        wait_for_clock(node)
        if node.map_version:
            print(f'map_version {node.map_version}')

        if a.scenario:
            from nav import scene
            from send_goal import teleport
            if a.scenario not in scene.SCENARIOS:
                ap.error(f'알 수 없는 시나리오 {a.scenario!r}; '
                         f'{sorted(scene.SCENARIOS)} 중 하나')
            teleport(a.scenario)
            # 텔레포트가 TF 와 costmap 에 반영될 시간. 1.5 s 로는 planner 가 낡은
            # pose 에서 경로를 만드는 일이 있었다 - 같은 시나리오인데 경로 길이가
            # 8.96 m 와 7.60 m 로 갈렸다. 주행 결과는 영향이 없지만(도착 판정은
            # goal_xy 로 한다) 측정값이 흔들리면 검증이 못 미더워진다.
            time.sleep(3.0)
            node.clear_costmap()
            goal_xy = list(scene.SCENARIOS[a.scenario][1])
            yaw = a.xy[2] if len(a.xy) > 2 else 0.0
        elif a.raw_path:
            pts = np.asarray(a.raw_path, float).reshape(-1, 2)
            goal_xy, yaw = list(pts[-1]), (a.xy[2] if len(a.xy) > 2 else 0.0)
        elif len(a.xy) >= 2:
            goal_xy, yaw = a.xy[:2], (a.xy[2] if len(a.xy) > 2 else 0.0)
        else:
            ap.error('목표가 필요하다: "X Y [YAW]" 또는 --scenario 또는 --raw-path')

        if a.raw_path:
            path = node.raw_path(np.asarray(a.raw_path, float).reshape(-1, 2))
            label = 'raw'
        else:
            path = node.plan(goal_xy, yaw)
            label = 'drive'

        res = node.navigate(path, goal_xy, yaw, label,
                            cancel_after=a.cancel_after, drop_hb=a.drop_heartbeat,
                            bad_version=a.bad_map_version, double=a.double_goal,
                            drop_loc=a.drop_localization)

        if a.rotate4:
            if res is None or res.status != _R.STATUS_SUCCEEDED:
                print('주행이 성공하지 않아 회전을 건너뛴다')
            else:
                fp = res.final_pose.pose
                here = [fp.position.x, fp.position.y]
                th = yaw_of(fp.orientation)
                # 90 도씩 네 번. path 는 현재 위치 한 점이고 yaw 만 바뀐다 - 탐색
                # 시스템이 관측 회전을 요청하는 형태 그대로다.
                for k in range(4):
                    th = wrap(th + math.pi / 2)
                    r = node.navigate(node.hold_path(here), here, th, 'rotate')
                    if r is None or r.status != _R.STATUS_SUCCEEDED:
                        print(f'회전 {k + 1}/4 실패, 중단')
                        break
                    fp = r.final_pose.pose
                    drift = math.hypot(fp.position.x - here[0], fp.position.y - here[1])
                    print(f'         회전 {k + 1}/4 위치 드리프트 {drift:.3f} m')
    finally:
        node.destroy_node()
        # spin 스레드가 이미 shutdown 을 거쳤으면 두 번 부르면 RCLError 가 난다.
        # Ctrl-C 마다 보이는 노이즈라 막는다.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
