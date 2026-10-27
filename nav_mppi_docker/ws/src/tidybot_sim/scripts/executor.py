#!/usr/bin/env python3
"""Execution core for the team-integration navigator.

State machine, monitors, and the /cmd_vel output. Deliberately knows nothing
about any action type: outcomes and phases come out as lowercase strings and
the frontend maps them to whatever athome_interfaces/NavigateToGoal declares.
That is what lets this file be written and tested before the .action arrives.

    IDLE --start()--> RUNNING --(arrival | monitor | cancel | fail)--> STOPPING --> IDLE

Every exit goes through STOPPING, which ramps the command to zero and waits for
/odom to agree the base has actually stopped. The team contract says the result
is only sent once the robot is standing still, so "stopped" is measured, not
assumed - that is the whole reason STOPPING is a state and not a line of code.

Frames. Pose comes from TF (global_frame -> base_frame) and nothing else; the
costmap and the path are both in global_frame, so unlike the FollowPath node
there is no path->odom conversion here. /odom is read for its twist only, as
the speed source for the stop test.
"""

import csv
import math
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import tf2_ros
from ament_index_python.packages import get_package_share_directory
import yaml
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from visualization_msgs.msg import Marker, MarkerArray


# ── states, phases, outcomes ─────────────────────────────────────────────────
# Plain strings on purpose. The frontend owns the integer encoding because the
# integers live in the .action file; see navigate_to_goal.py's REASON table.
IDLE, RUNNING, STOPPING = 'idle', 'running', 'stopping'
PHASES = ('idle', 'driving', 'rotating', 'stopping')
# Terminal outcomes. 'reached' and 'canceled' are successes of a sort; the rest
# are the failure reasons the team spec enumerates.
OUTCOMES = ('reached', 'canceled', 'blocked', 'no_progress', 'timeout',
            'localization_lost', 'internal')

# Executor parameters, declared on the frontend node so a launch file or a yaml
# can reach them. Values that belong to the robot rather than to this node live
# in demo.yaml instead (robot_radius, tolerances, NO_PROGRESS, timeouts).
# 진단 CSV 가 쌓이는 곳. colcon build --symlink-install 이면 resolve() 가 소스 트리로
# 되돌아가 src/tidybot_sim/results/ 에 쌓인다 (mppi_controller.py 와 같은 방식).
# 복사 설치면 install 공간에 쌓이는데, 로그라 동작에는 지장이 없다. 이 프로젝트는
# --symlink-install 로 빌드한다.
RESULTS = Path(__file__).resolve().parent.parent / 'results'
LOG_FIELDS = ['t', 'phase', 'x', 'y', 'th', 'ux', 'uy', 'uw', 'v_meas', 'w_meas',
              # 진단용: lookahead 목표점과 경로 최근접 인덱스. 우회 중 진척이 뒤로
              # 튀는지, lookahead 가 단조 증가하는지 사후에 보려면 이 셋이 필요하다.
              'wp_x', 'wp_y', 'near_i',
              'rem', 'eta', 'lam', 'n_hit', 'fallback', 'cm_ms', 'mppi_ms',
              'heading_ref',                   # 설정값. solve 와 무관하게 항상 찍는다
              # 비용장. MPPI 가 경로와 싸우는 것을 '비싸다'까지만 알았고 어디가 비싼지는
              # 몰랐다. c_seg_max 가 '경로가 벽에 붙었는지'를 직접 잡는 값이다.
              'c_robot', 'c_wp', 'c_seg_max',
              'lam_hit', 'S_min', 'S_p50', 'S_max',
              'best_goal', 'best_obs', 'best_head', 'best_ctrl',
              'w_fwd', 'w_left', 'w_back', 'w_right',
              # yaw_err 는 비용에 들어간 오차, yaw_ref 는 그 기준값 자체다.
              'yaw_err', 'yaw_ref', 'w_odom', 'tf_age',
              # 목표 셀 기준 단측 벌점의 기준선. 램프가 실제로 걸렸는지 로그로
              # 확인할 수 없으면 이 수정의 효과를 측정할 수 없다.
              'obs_ref',
              # Biased-MPPI 샘플 블록별 Sum(w) 와 IS 항 평균. biased 가 꺼져 있거나
              # _ancillary 가 None 을 내면 w_gauss 만 1.0 이고 나머지는 빈 칸이다.
              # 모드 평균화 판정은 |w_sneg - w_spos| 가 작으면서 ||u|| 도 작은 tick 을
              # 세는 것이다 -- 두 측면 후보가 비기면 갱신이 그 평균이라 지령이 줄어든다.
              'w_gauss', 'w_path', 'w_sneg', 'w_spos',
              'is_gauss', 'is_path', 'is_sneg', 'is_spos',
              # 블록별 lethal 마스킹 비율. Sum(w)=0 이 비용 패배인지 전원 탈락인지
              # 가른다 -- 그것 없이 ancillary 의 0 을 비용 패배로 잘못 읽었다.
              'hit_gauss', 'hit_path', 'hit_sneg', 'hit_spos']

# step() 이 돈 tick 에만 의미가 있는 필드. solve 없이 행이 나가면 **직전 tick 의 값**이
# 실려 조용히 틀린 진단이 된다. eta/lam/n_hit/fallback 도 같은 성질인데 지금까지
# 무방비였으므로 함께 넣는다. 현 구조에서는 기록부가 step() 뒤에만 도달하지만 그
# 전제는 코드가 바뀌면 깨지고, 깨진 것이 보이지 않는 종류의 오류다.
DIAG_FIELDS = ('eta', 'lam', 'n_hit', 'fallback',
               'c_robot', 'c_wp', 'c_seg_max', 'lam_hit', 'S_min', 'S_p50', 'S_max',
               'best_goal', 'best_obs', 'best_head', 'best_ctrl',
               'w_fwd', 'w_left', 'w_back', 'w_right', 'yaw_err', 'yaw_ref', 'obs_ref',
               'w_gauss', 'w_path', 'w_sneg', 'w_spos',
               'is_gauss', 'is_path', 'is_sneg', 'is_spos',
               'hit_gauss', 'hit_path', 'hit_sneg', 'hit_spos')
EMPTY_DIAG = {k: '' for k in DIAG_FIELDS}

# 기본값이 **없는** 필수 파라미터. 파라미터 파일이 안 들어오면 노드가 기동 단계에서
# 죽는다 -- 조용히 코드 기본값으로 도는 것보다 낫다. 실제로 sim.launch.py 가 엉뚱한
# 파일을 넘겨 fastlio_mock 이 기본값으로 돌았고, 실패해야 할 테스트가 통과했다.
#
# 대상은 '틀린 값으로도 그럴듯하게 도는' 것들이다: 프레임, 토픽, 주기, 감시 임계.
# 조율값(lookahead, 정지 임계, 가중치, 로깅)은 기본값을 둔다.
REQUIRED = [
    ('solve_frequency', Parameter.Type.DOUBLE),
    ('publish_frequency', Parameter.Type.DOUBLE),
    ('global_frame', Parameter.Type.STRING),
    ('base_frame', Parameter.Type.STRING),
    # 속도 추정 전용 프레임. pose 는 계속 global_frame -> base_frame 으로 읽지만,
    # 속도는 velocity_frame -> base_frame 차분으로 잰다.
    #
    # 실기에서 map -> base_link 에는 ICP 정합 보정이 섞인다: 로봇이 멈춰 있어도
    # ~1 Hz 로 2~3 cm 씩 계단 점프가 생기고, 0.3 s 창으로 차분하면 0.056 m/s 가
    # 나와 stop_speed 0.02 를 넘는다 (실측). 그러면 모든 goal 이 stopped=false 로
    # 끝난다. camera_init -> base_link 에는 그 점프가 없다.
    ('velocity_frame', Parameter.Type.STRING),
    ('static_costmap_topic', Parameter.Type.STRING),
    ('local_costmap_topic', Parameter.Type.STRING),
    ('cmd_vel_topic', Parameter.Type.STRING),
    ('input_wait', Parameter.Type.DOUBLE),
    ('static_stale', Parameter.Type.DOUBLE),
    ('local_stale', Parameter.Type.DOUBLE),
    ('tf_timeout', Parameter.Type.DOUBLE),
]

PARAMS = [
    # 실기 static 은 /mppi_static_costmap/costmap 이다. 시뮬에서는 planner_server 의
    # global_costmap 이 그 역할을 하는데, Costmap2DROS 가 부모의 리매핑을 물려받지
    # 않아 토픽 이름을 바꿀 수 없다. 실기 연결 시 이 파라미터만 바꾸면 된다.
    # 입력 감시. 수락 직후 input_wait 안에 TF, static, local 이 모두 유효하지 않으면
    # 정지 후 abort 한다. 그 뒤로는 각 stale 임계가 다스린다. 조용히 기다리다
    # TIMEOUT 으로 끝나는 경로를 없애는 것이 목적이다.
    # costmap stale 로 죽을 때 같은 tick 의 tf_age 가 이 값을 넘으면 internal 이 아니라
    # localization_lost 로 보고한다. 둘은 무선 링크가 끊기면 **함께** 죽는데, local
    # costmap 이 3 Hz 라 정상 상태에서도 0.33 s 밀려 있어 local_stale 이 tf_timeout 보다
    # 항상 먼저 터진다 (실측 3/3: 그 시점 tf_age 1.438 / 1.626 / 1.728 로 2.0 미달).
    # 그러면 위치 상실이 영영 costmap 문제로 보고되고, 클라이언트의 '위치 상실은 사람이
    # 확인' 규칙에 걸리지 않는다. tf_timeout 을 재분류 임계로 쓸 수는 없다 -- 그 경계에서
    # _pose() 가 이미 None 을 내므로 재분류가 발동할 창이 없다. 0.5 는 측위 담당이 권고한
    # 값이고, 0.5 s 넘게 새 TF 가 없는 상태는 실제로 위치를 잃은 것이다.
    # 0 이면 재분류하지 않는다 = 이전과 같은 reason. 실기 프로필만 옵트인한다.
    ('tf_stale_hint', 0.0),             # s (실기 0.5)
    # NO_INFORMATION (-1) handling. 'free' matches the costmap's own
    # default_value and the global path has already vetted the topology;
    # 'lethal' is the conservative reading for a map with real unknown space.
    ('unknown', 'free'),                # 'free' | 'lethal'
    ('lookahead_dist', 0.75),           # carrot distance along the path
    ('goal_switch_dist', 0.75),         # below this the carrot becomes the goal itself
    # STOPPING. decel_limit ramps the published command; the base is a velocity
    # source, so the ramp IS the deceleration. stop_speed/stop_hold are read
    # against /odom's twist.
    ('decel_limit', 1.0),               # m/s^2 on the published command
    # 각속도는 따로 둔다. 하나로 묶으면 1.0 rad/s^2 로 램프하는데, 램프가 ω 에서 0
    # 까지 쓸어내는 각도는 ω^2/(2*decel) 이라 실측 ω 0.341 rad/s 에서 3.33 deg 가 된다.
    # 탐색 회전의 허용 오차가 +-5.3 deg (= FOV 70.6 와 60 deg 간격의 겹침 절반) 이므로
    # 그 하나로 예산의 63 % 를 쓴다. 베이스의 각가속 한계는 3.14 rad/s^2 이고, 3.0 이면
    # 같은 ω 에서 1.11 deg 로 떨어진다. 선형은 건드리지 않는다 -- 병진 글라이드는
    # xy_tolerance 0.15 m 안에 이미 들어온다.
    # 0 이면 decel_limit 을 쓴다 = 이전과 완전히 같은 동작. 실기 프로필만 옵트인한다.
    ('decel_limit_w', 0.0),             # rad/s^2 on the published command; 0 -> decel_limit
    # 속도는 TF pose 차분으로 잰다. /odom 의 twist 는 쓰지 않는다 -- FAST-LIO 가
    # twist 를 채우지 않으므로, 그걸 쓰면 시뮬에서만 동작하는 정지 판정이 된다.
    # 창 자체가 필터다: 10 Hz 입력이면 0.3 s 안에 표본 3개가 들어온다.
    ('vel_window', 0.3),                # s, 차분 구간
    ('stop_speed', 0.02),               # m/s
    ('stop_yaw_rate', 0.05),            # rad/s, 제자리 회전을 정지로 오판하지 않게 따로 둔다
    ('stop_hold', 0.2),                 # s below the thresholds before calling it stopped
    ('stop_timeout', 3.0),              # s; past this, report stopped=False rather than hang
    # BLOCKED 를 내는 두 조건이 공유한다: (a) 전 rollout 이 lethal, (b) carrot 후보가
    # carrot_skip_max 안에 없음. 둘 다 '앞이 막혔다'는 같은 뜻이고 구분은 detail 이 한다.
    ('blocked_time', 3.0),              # s
    # 경로가 막혔을 때 carrot 을 앞으로 건너뛸 수 있는 최대 경로 길이. 0.35 m 상자의
    # 경로상 차단 구간이 약 1.1 m 이므로 2.0 m 면 넘긴다.
    ('carrot_skip_max', 2.0),           # m
    # Rotation mode: the same cost terms, different weights. w_goal holds the
    # position (the goal IS the current position), w_head does the turning.
    ('rot_w_head', 40.0),
    ('rot_w_goal', 60.0),
    # 최종 접근 ω 캡. MPPI 는 목표 yaw 근처에서 감속하지 않는다 -- 멈추기로 결정하는
    # 순간에도 ω 가 순항에 붙어 있다 (실측: 마지막 uw 0.341 vs 순항 0.357). 허용 오차가
    # 14.3 deg 일 때는 드러나지 않았지만 5.3 deg 에서는 결정 이후 쓸어내는 각도가
    # 예산을 넘긴다. 그 각도는 램프 ω^2/(2*decel_limit_w) 와 판정 지연 ω*0.17 의 합이고
    # (0.17 s = TF 전송 0.07 + tick 0.1), ω 를 0.20 으로 묶으면 0.38 + 1.95 = 2.3 deg 다.
    # 둘 중 하나라도 0 이면 캡을 끄고 이전과 완전히 같게 동작한다. 실기 프로필만
    # 옵트인한다 -- 시뮬 A/B 의 저장된 결과와 비교 가능해야 한다.
    ('yaw_approach', 0.0),              # rad, 이 안에서 캡을 건다 (실기 0.26 = 약 15 deg)
    ('w_approach', 0.0),                # rad/s, 그 구간의 |ω| 상한 (실기 0.20)
    ('viz', True),                      # rollout/best-trajectory markers
    ('viz_samples', 24),
    ('log_timing', True),
    ('log_prefix', 'team'),             # results/<prefix>_<goal>.csv
    # 주행 중 heading 기준. 'goal' 이 기본이고, 그때 동작은 이전과 완전히 같다
    # (yaw_ref == goal 의 최종 yaw 가 항상 성립한다). 'path' 면 캐럿 지점의 경로 접선
    # yaw 를 쓰고 goal_switch_dist 안에서 목표 yaw 로 전환한다. 비용장 변경 효과와
    # 섞이지 않게 **이 값만 바꿔 A/B** 하도록 분리해 두었다.
    ('heading_ref', 'goal'),            # 'goal' | 'path'
    # 접선을 재는 현의 길이. 한 세그먼트(= 격자 0.05 m)로 재면 A* 경로의 계단 때문에
    # 대각선 구간에서 0 도와 90 도를 번갈아 낸다. 0.30 m 는 6 칸이라 계단이 평균된다.
    ('tangent_span', 0.30),             # m
    # Biased-MPPI (arXiv:2401.09241). false 면 ancillary 를 아예 넘기지 않고, 그때 MPPI 는
    # 비트 단위로 예전과 같은 결과를 낸다. A/B 는 이 한 값만 바꿔서 한다.
    ('biased', False),
    # 측면 기동의 횡방향 ramp: h(t) = min(1, t / (anc_ramp_frac * (T-1))).
    # 0.5 면 T=25 에서 12 스텝(1.2 s)에 full 에 닿고 지평선 끝까지 유지한다. 계단
    # (frac -> 0)은 w_dir 벌점에 바로 걸려 탈락하고, 선형(frac=1)은 지평선 안에 우회를
    # **끝내지 못해** 비용 평가가 '반쯤 비킨 궤적'만 본다.
    ('anc_ramp_frac', 0.5),
    # ω 추종 진단용 구독. '' 이면 구독하지 않는다 -- 실기에 twist 가 채워져 오는지
    # 확인되지 않았고(FAST-LIO 는 안 채운다), 없는 토픽을 기본값으로 두면 조용히
    # 빈 칸만 쌓인다. TF yaw 차분으로 만든 w_meas 는 |ω| ~ 0.07 rad/s 를 판정할
    # 해상도가 없다 (실측: w_cmd 와 부호가 50 % 반대, n=2419).
    ('odom_topic', ''),
]


def read_params(node, required=(), optional=()):
    """선언하고 바로 읽는다. 필수 항목이 비어 있으면 여기서
    ParameterUninitializedException 으로 죽는다 -- 그것이 목적이다."""
    items = list(REQUIRED) + list(required) + list(PARAMS) + list(optional)
    node.declare_parameters('', items)
    return {n: node.get_parameter(n).value for n, _ in items}


def load_robot_config(path):
    """demo.yaml -> dict. Absolute path, or resolved against this package's share.

    share 로 찾는다. __file__ 기준으로 올라가면 colcon 이 symlink-install 일 때만
    맞고, 복사 설치에서는 install/tidybot_sim/lib/config/ 를 보다가 파일이 없다며
    노드가 죽는다 -- 깨끗한 빌드 한 번에 드러났다.
    """
    p = Path(path)
    if not p.is_absolute():
        p = Path(get_package_share_directory('tidybot_sim')) / 'config' / p.name
    d = yaml.safe_load(p.read_text())
    missing = {'robot_radius', 'xy_tolerance', 'yaw_tolerance', 'no_progress_time',
               'no_progress_distance', 'timeout_drive', 'timeout_rotate'} - set(d)
    assert not missing, f'{p} is missing {sorted(missing)}'
    return d


def wrap(a):
    """-> [-pi, pi)."""
    return (a + math.pi) % (2 * math.pi) - math.pi


class CostmapGrid:
    """The three members of nav/grid.py's Grid that mppi.py reads, over an
    OccupancyGrid in the global frame.

    ponytail: this duplicates mppi_controller.py's CostmapGrid. The difference
    is real - unknown handling is a parameter here, and the baseline's class
    hardcodes -1 -> free - but the two should become one when the FollowPath
    frontend is folded into this executor. Until then mppi_controller.py stays
    frozen so its benchmark numbers keep meaning what they meant.

    Scale (getting this wrong is silent and fatal):

        OccupancyGrid  99, 100  -> LETHAL 10.0     (costmap_2d 253/254)
                       1..98    -> (0, 1]          the inflated band
                       0        -> 0               free
                       -1       -> `unknown`       NO_INFORMATION

    The inflated band tops out at 98, not 10: dividing the whole range by 10
    makes it ten times hotter than the one w_obs = 20.0 was tuned against.
    """

    LETHAL = 10.0
    INSCRIBED = 99

    def __init__(self, msg: OccupancyGrid, expect_frame, unknown='free', outside=LETHAL):
        assert msg.header.frame_id == expect_frame, (
            f'costmap is in {msg.header.frame_id!r}, expected {expect_frame!r}; '
            'the rollouts and the cost lookup would be in different frames')
        info = msg.info
        self.origin = np.array([info.origin.position.x, info.origin.position.y])
        self.res = info.resolution
        raw = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
        self.outside = outside
        self._labels = None             # labels() 가 처음 불릴 때만 만든다
        unk = self.LETHAL if unknown == 'lethal' else 0.0
        self.cost = np.where(raw < 0, unk,
                             np.where(raw >= self.INSCRIBED, self.LETHAL,
                                      raw / (self.INSCRIBED - 1)))

    def cell(self, pts):
        """pts: (..., 2) -> (row, col, 창 안인가)."""
        idx = np.floor((pts - self.origin) / self.res).astype(np.int32)
        h, w = self.cost.shape
        c, r = idx[..., 0], idx[..., 1]
        return (np.clip(r, 0, h - 1), np.clip(c, 0, w - 1),
                (c >= 0) & (c < w) & (r >= 0) & (r < h))

    def labels(self):
        """자유 셀(cost < LETHAL)의 8-연결 성분. 0 은 막힌 셀이다.

        carrot 을 건너뛰어야 할 때만 계산하고, grid 객체와 수명을 같이 한다 --
        local 메시지가 새로 오면 grid 가 새로 만들어지므로 자연히 메시지당 1 회다.
        """
        if self._labels is None:
            from scipy.ndimage import label
            self._labels, _ = label(self.cost < self.LETHAL,
                                    structure=np.ones((3, 3), dtype=int))
        return self._labels

    def lookup(self, pts):
        """pts: (..., 2) world xy in the costmap frame -> (...,) cost."""
        idx = np.floor((pts - self.origin) / self.res).astype(np.int32)
        h, w = self.cost.shape
        c, r = idx[..., 0], idx[..., 1]
        inside = (c >= 0) & (c < w) & (r >= 0) & (r < h)
        # 범위 밖 값은 grid 마다 다르다.
        #   static  -> LETHAL. 측량되지 않은 바깥으로 나간 rollout 이 싸 보이면 안 된다.
        #   local   -> 0. rolling window 바깥은 '모른다'이지 '막혔다'가 아니고,
        #              max() 로 합칠 때 static 의 값이 살아야 한다.
        return np.where(inside,
                        self.cost[np.clip(r, 0, h - 1), np.clip(c, 0, w - 1)],
                        self.outside)


class PairGrid:
    """static 과 local 을 합쳐 mppi.py 가 쓰는 lookup() 하나로 낸다.

    셀 대응은 **grid 마다 독립 변환**이다. rolling window 의 origin 이 static grid 와
    셀 경계로 맞는다는 보장이 없어서, 인덱스 오프셋을 쓰면 반 셀씩 밀린 cost 를
    조용히 읽게 된다 -- 장애물이 2.5 cm 옮겨 보이는 증상은 디버깅으로 못 잡는다.
    비용은 측정값으로 0.21 ms (조회 0.18 -> 0.39 ms, solve 예산 100 ms).
    """

    def __init__(self):
        self.static = self.local = None

    def lookup(self, pts):
        c = self.static.lookup(pts)
        if self.local is not None:
            c = np.maximum(c, self.local.lookup(pts))
        return c


class Executor:
    """Drives one goal at a time. The frontend owns the action; this owns the robot."""

    def __init__(self, node, mppi, p, robot):
        self.node, self.mppi, self.p, self.robot = node, mppi, p, robot
        # 오타가 조용히 'goal' 로 떨어지면 A/B 의 한쪽이 기준과 같아진 것을 모른 채
        # 결과를 읽게 된다 -- MPPI.set_weights 의 화이트리스트와 같은 이유다.
        assert p['heading_ref'] in ('path', 'goal'), \
            f"heading_ref must be 'path' or 'goal', got {p['heading_ref']!r}"
        self.log = node.get_logger()
        self.lock = threading.Lock()

        self.state, self.phase = IDLE, 'idle'
        self._goal = None                   # dict set by start()
        self._outcome = None                # set on entering STOPPING, read by take_outcome()
        self._detail = ''                   # 왜 멈췄는지. 프론트엔드가 Result.detail 로 싣는다
        self._done = None                   # (outcome, final_pose, stopped) once IDLE again

        self._u = np.zeros(3)               # latest solved command, GLOBAL frame
        self._cmd = np.zeros(3)             # what _publish last put on the wire, global frame
        self._moved = False
        # 베이스가 지금 멈춰 있다고 '측정으로' 확인된 상태인가.
        # 노드 기동 직후에는 알 수 없으므로 False 로 시작한다.
        self._rest_confirmed = False
        # 두 costmap. 변환은 각 콜백에서 메시지당 1 회, solve 경로에는 없다.
        # _t 는 수신 시각(노드 clock) -- stale 판정의 기준이다.
        self.grid = PairGrid()
        self._static_t = self._local_t = None
        self._tf_age = float('nan')         # _pose() 가 매 tick 채운다
        self._w_odom = None                 # odom_topic 이 비어 있으면 영영 None
        self._diag = None                   # 이번 tick 의 진단. step() 직전에 비운다
        # TF pose 표본 (stamp_s, pose) -- 속도 추정의 유일한 입력.
        self._samples, self.vlock = deque(maxlen=64), threading.Lock()
        self._csv = self._csv_f = None      # goal 단위 스트리밍 기록
        self._pose_xy_th = None             # last good TF read, for the final_pose report

        # The drive-mode weights are whatever MPPIConfig set, captured now so the
        # switch back out of rotation is exact rather than a second copy of the
        # numbers.
        self._drive_w = {k: getattr(mppi, k) for k in mppi.WEIGHTS}
        self._rot_w = {'w_head': p['rot_w_head'], 'w_goal': p['rot_w_goal'], 'w_dir': 0.0}

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)

        self.group = ReentrantCallbackGroup()

        # nav2_costmap_2d 의 발행자와 맞춰야 한다. 안 맞으면 조용히 한 장도 안 온다.
        cm_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(OccupancyGrid, p['static_costmap_topic'],
                                 self._on_static, cm_qos, callback_group=self.group)
        node.create_subscription(OccupancyGrid, p['local_costmap_topic'],
                                 self._on_local, cm_qos, callback_group=self.group)
        if p['odom_topic']:
            # 진단 전용이라 기본 QoS 로 둔다. import 를 여기서 하는 것은 이 메시지가
            # 선택적 의존이기 때문이다 -- 토픽을 안 쓰면 타입도 필요 없다.
            from nav_msgs.msg import Odometry
            node.create_subscription(Odometry, p['odom_topic'],
                                     self._on_odom, 10, callback_group=self.group)
        self.cmd_pub = node.create_publisher(Twist, p['cmd_vel_topic'], 10)
        self.viz_pub = node.create_publisher(MarkerArray, '~/rollouts', 1)

        node.create_timer(1.0 / p['solve_frequency'], self._solve, callback_group=self.group)
        node.create_timer(1.0 / p['publish_frequency'], self._publish, callback_group=self.group)

    # ── frontend API ─────────────────────────────────────────────────────────

    def start(self, path_xy, goal_xy, goal_yaw, xy_tol, yaw_tol, timeout):
        """Begin a goal. path_xy: (N, 2) in global_frame. Returns False if busy."""
        with self.lock:
            if self.state != IDLE:
                return False
            # A one-point path is the in-place observation turn the exploration
            # system asks for after arriving: same action, position unchanged,
            # yaw different. No separate rotation controller - just the other
            # weight set (see nav/mppi.py set_weights).
            rotating = len(path_xy) <= 1
            self._goal = dict(
                path=np.asarray(path_xy, float).reshape(-1, 2),
                xy=np.asarray(goal_xy, float), yaw=float(goal_yaw),
                xy_tol=float(xy_tol), yaw_tol=float(yaw_tol),
                t0=self.node.get_clock().now(),
                timeout=float(timeout) if timeout > 0 else
                    self.robot['timeout_rotate' if rotating else 'timeout_drive'],
                prog_d=None, prog_t=None, blocked_t=None, carrot_block_t=None)
            self._set_mode('rotating' if rotating else 'driving')
            self._outcome, self._done, self._detail = None, None, ''
            self._u[:] = 0.0            # 입력을 기다리는 동안 /cmd_vel 은 0 으로 나간다
            self._moved = False         # 이 goal 에서 0 이 아닌 명령을 낸 적이 있는가
            # 이전 goal 의 종료 위치를 지운다. 남겨두면 이 goal 의 첫 feedback 이
            # 그 낡은 pose 로 remaining 을 재서, 0 보다 나쁜 '그럴듯한 틀린 값'이
            # 나간다 (측정: corridor 경로에 clutter 종료점을 물려 7.78 m).
            self._pose_xy_th = None
            self.state = RUNNING
        # 락 밖에서 연다: 파일 I/O 가 solve 타이머를 붙잡을 이유가 없다.
        self._open_log()
        self.log.info(f'start: {len(path_xy)} poses, goal ({goal_xy[0]:.2f}, {goal_xy[1]:.2f}, '
                      f'{goal_yaw:+.2f}), phase {self.phase}, timeout {self._goal["timeout"]:.0f}s')
        return True

    def cancel(self):
        self._stop('canceled')

    def fail(self, reason):
        """Frontend-detected failure (heartbeat loss, map mismatch mid-flight)."""
        assert reason in OUTCOMES, reason
        self._stop(reason)

    def take_outcome(self):
        """(outcome, final_pose, stopped, detail) once the base is standing still, else None.

        Returned exactly once; the frontend polls this from its own timer so the
        result is sent after the robot stops, which is the team contract.
        """
        with self.lock:
            d, self._done = self._done, None
            return d

    def feedback(self):
        """(pose (3,), remaining_distance, phase) - safe to call any time."""
        with self.lock:
            x = self._pose_xy_th
            g = self._goal
        # 캐시는 solve 가 돌 때만 갱신된다. 비어 있거나(goal 의 첫 feedback, start()
        # 가 지운 직후) IDLE 이면(시작도 못 하고 거절된 goal 의 final_pose) 직접
        # 읽는다. 둘 다 답이 '지금 로봇이 어디 있는가'여야 하는 자리다. RUNNING 중에는
        # solve 가 100 ms 마다 채우므로 여기서 또 읽지 않는다.
        if x is None or self.state == IDLE:
            fresh = self._pose()        # ndarray 라 `or` 를 쓰면 truthiness 로 터진다
            if fresh is not None:
                x = fresh
        if g is None:
            return x, 0.0, self.phase
        if x is None:
            # TF 자체가 없다. 위치는 모르지만 경로가 얼마나 남았는지는 아니까,
            # 그 값이 0 보다 정직하다.
            return None, self._path_len(g['path']), self.phase
        return x, self._remaining(x[:2], g['path']), self.phase

    # ── callbacks: store only ────────────────────────────────────────────────

    def _on_static(self, msg):
        """OccupancyGrid -> numpy 변환은 **여기서** 메시지당 1 회만 한다.

        실제 맵 크기(747x1316 = 983k 셀)로 오프라인 측정한 변환 시간이 p50 3.55 ms
        인데, 1 Hz 이므로 0.36 % 부하다. solve 안에서 하면 10 Hz 로 같은 일을 열 번
        하게 된다 -- 기준선 노드(mppi_controller.py)가 그렇게 하고 있다.
        """
        try:
            self.grid.static = CostmapGrid(msg, self.p['global_frame'], self.p['unknown'],
                                           outside=CostmapGrid.LETHAL)
            self._static_t = self.node.get_clock().now().nanoseconds / 1e9
        except AssertionError as e:
            self.log.error(str(e), throttle_duration_sec=5.0)

    def _on_local(self, msg):
        """rolling window. origin 이 매 메시지 바뀌므로 그때그때 다시 만든다.
        범위 밖은 0 -- '모른다'이지 '막혔다'가 아니다."""
        try:
            self.grid.local = CostmapGrid(msg, self.p['global_frame'], self.p['unknown'],
                                          outside=0.0)
            self._local_t = self.node.get_clock().now().nanoseconds / 1e9
        except AssertionError as e:
            self.log.error(str(e), throttle_duration_sec=5.0)

    def _on_odom(self, msg):
        """구동계/FAST-LIO 의 ω. **진단 전용이고 제어에는 쓰지 않는다.**"""
        self._w_odom = float(msg.twist.twist.angular.z)

    def _bad_inputs(self):
        """[(이름, 토픽, 나이|None)] -- 비어 있으면 입력이 모두 유효하다."""
        now = self.node.get_clock().now().nanoseconds / 1e9
        out = []
        for name, t, lim, topic in (
                ('static', self._static_t, self.p['static_stale'], self.p['static_costmap_topic']),
                ('local', self._local_t, self.p['local_stale'], self.p['local_costmap_topic'])):
            if t is None:
                out.append((name, topic, None))
            elif now - t > lim:
                out.append((name, topic, now - t))
        return out

    # ── internals ────────────────────────────────────────────────────────────

    def _open_log(self):
        """goal 하나당 파일 하나. log_prefix 로 gt 와 fastlio_mock 실행을 구분한다."""
        self._close_log()
        if not self.p['log_timing']:
            return
        RESULTS.mkdir(parents=True, exist_ok=True)
        path = RESULTS / f"{self.p['log_prefix']}_{int(time.time())}.csv"
        self._csv_f = path.open('w', newline='')
        self._csv = csv.DictWriter(self._csv_f, fieldnames=LOG_FIELDS)
        self._csv.writeheader()
        self.log.info(f'logging to {path}')

    def _close_log(self):
        if self._csv_f is not None:
            self._csv_f.close()
        self._csv = self._csv_f = None

    def _set_mode(self, phase):
        assert phase in PHASES, phase
        self.phase = phase
        self.mppi.set_weights(**(self._rot_w if phase == 'rotating' else self._drive_w))

    def _pose(self):
        """(x, y, theta) in global_frame, or None if TF is missing or stale."""
        try:
            # Time() is "the latest available". Control must not run on an old
            # pose, so staleness is caught by reading the stamp afterwards
            # rather than by asking for a transform in the past.
            t = self.tf_buffer.lookup_transform(
                self.p['global_frame'], self.p['base_frame'], Time(),
                timeout=Duration(seconds=0.05))
        except tf2_ros.TransformException as e:
            self.log.warn(f'TF: {e}', throttle_duration_sec=2.0)
            return None
        stamp = Time.from_msg(t.header.stamp)
        age = (self.node.get_clock().now() - stamp).nanoseconds / 1e9
        self._tf_age = age      # tick 마다 CSV 로 나간다. 무선 링크에서 p99 1.36 s 였다
        if age > self.p['tf_timeout']:
            self.log.warn(f'TF is {age:.2f}s old', throttle_duration_sec=2.0)
            return None
        q, v = t.transform.rotation, t.transform.translation
        # yaw 만 남기고 z, roll, pitch 는 버린다. 센서 장착 기울기는 static
        # imu_link -> base_link 역변환이 이미 상쇄했으므로, 여기 들어오는 것은
        # 베이스의 자세다 (fastlio_mock 의 왕복 검산: pitch 0.1 rad 장착에서 yaw
        # 오차 4e-16 rad).
        return np.array([v.x, v.y,
                         math.atan2(2 * (q.w * q.z + q.x * q.y),
                                    1 - 2 * (q.y ** 2 + q.z ** 2))])

    def _vel_sample(self):
        """velocity_frame -> base_frame 을 한 번 읽어 속도 추정용 표본에 넣는다.

        pose 와 **다른 프레임**을 쓰는 것이 요점이다 (REQUIRED 의 velocity_frame 주석).
        stale 판정은 pose 와 같은 tf_timeout 을 쓴다. 다만 여기서 실패해도 abort 하지
        않는다 -- 같은 TF 사슬의 윗부분을 _pose() 가 이미 보고 있고, 표본이 안 쌓이면
        _velocity() 가 None 을 내어 '모른다'로 흐르기 때문이다.
        """
        try:
            t = self.tf_buffer.lookup_transform(
                self.p['velocity_frame'], self.p['base_frame'], Time(),
                timeout=Duration(seconds=0.05))
        except tf2_ros.TransformException:
            return
        stamp = Time.from_msg(t.header.stamp)
        if (self.node.get_clock().now() - stamp).nanoseconds / 1e9 > self.p['tf_timeout']:
            return
        q, v = t.transform.rotation, t.transform.translation
        pose = np.array([v.x, v.y,
                         math.atan2(2 * (q.w * q.z + q.x * q.y),
                                    1 - 2 * (q.y ** 2 + q.z ** 2))])
        # TF 는 10 Hz 로 오는데 이 함수는 publish 주기로 불린다. 같은 stamp 를 다시
        # 넣으면 dt = 0 인 쌍이 생기고 창이 중복으로 가득 찬다.
        ts = stamp.nanoseconds / 1e9
        with self.vlock:
            if not self._samples or self._samples[-1][0] != ts:
                self._samples.append((ts, pose))

    def _velocity(self):
        """(|v_xy|, |omega|) from TF differencing, or None when unknown.

        None 은 '모른다'이고 0 이 아니다. /odom 의 twist 를 쓰지 않는 이유는
        FAST-LIO 가 그것을 채우지 않기 때문이고, 모를 때 0 을 돌려주면 위치를
        잃은 로봇이 '정지했다'로 보고된다.
        """
        now = self.node.get_clock().now().nanoseconds / 1e9
        w = self.p['vel_window']
        with self.vlock:
            s = list(self._samples)
        # 최신 표본이 창보다 오래됐으면 지금 속도를 말할 근거가 없다.
        if not s or now - s[-1][0] > w:
            return None
        t1, p1 = s[-1]
        inside = [(t, q) for t, q in s if t1 - t <= w]
        if len(inside) < 2:
            return None
        t0, p0 = inside[0]
        dt = t1 - t0
        if dt <= 0:
            return None
        return (float(np.linalg.norm(p1[:2] - p0[:2]) / dt),
                abs(wrap(p1[2] - p0[2]) / dt))

    @staticmethod
    def _path_len(path):
        """경로 전체 길이. 한 점이면 0 -- 제자리 회전에 남은 거리는 실제로 0 이다."""
        return float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()) if len(path) > 1 else 0.0

    def _nearest(self, xy, path):
        return int(np.argmin(np.linalg.norm(path - xy, axis=1))) if len(path) else 0

    def _remaining(self, xy, path):
        """Distance left along the path. One point -> straight line to it."""
        if len(path) < 2:
            return float(np.linalg.norm(path[-1] - xy)) if len(path) else 0.0
        i = int(np.argmin(np.linalg.norm(path - xy, axis=1)))
        seg = float(np.linalg.norm(np.diff(path[i:], axis=0), axis=1).sum())
        return seg + float(np.linalg.norm(path[i] - xy))

    def _waiting_for_dynamic(self):
        """Hook for the pedestrian-prediction input that is out of scope today.

        When it lands, return True while the base is deliberately holding for a
        moving obstacle: NO_PROGRESS must not fire on a robot that is correctly
        waiting. Nothing else in this file needs to change.
        """
        return False

    def _stop(self, outcome, detail=''):
        """Enter STOPPING with a decided outcome. Idempotent - the first caller wins."""
        with self.lock:
            if self.state != RUNNING:
                return
            self._outcome, self._detail = outcome, detail
            self.state, self._stop_t0, self._below_t = STOPPING, time.monotonic(), None
            self.phase = 'stopping'
        # 두 줄로 나뉘어 있는 것이 스타일이 아니라 요구사항이다. rclpy 의 로거는
        # 호출 지점(소스 위치)별로 심각도를 캐싱해서, 한 줄에서 info 를 쓴 뒤 warn 을
        # 쓰면 ValueError: Logger severity cannot be changed between calls 를 던진다.
        # 그 예외가 fail() 밖으로 빠져나가면 액션이 기본 Result(status=0)로 abort 되어,
        # 실패한 goal 이 "status 0" 이라는 금지값으로 보고된다. 실제로 당했다.
        if outcome in ('reached', 'canceled'):
            self.log.info(f'stopping: {outcome}')
        else:
            self.log.warn(f'stopping: {outcome}{(" -- " + detail) if detail else ""}')

    def _solve(self):
        if self.state != RUNNING:
            return
        with self.lock:
            g = self._goal
        if g is None:
            return
        t0 = time.perf_counter()
        # TF 를 **costmap 보다 먼저** 본다. 위치를 잃으면 costmap 발행자도 map->base_link
        # 를 못 구해 함께 멈추는데, costmap 을 먼저 보면 그 부재가 TF 부재를 가려
        # 위치 상실이 영영 reason 7 로 보고되지 않는다. 실제로 그렇게 당했다.
        x = self._pose()
        bad = self._bad_inputs()
        if x is None or bad:
            # 수락 직후 input_wait 동안은 기다린다. 그 전에는 '한 번도 안 왔음'과
            # '늦었음'을 구분할 수 없고, 입력이 갖춰질 시간도 필요하다.
            elapsed0 = (self.node.get_clock().now() - g['t0']).nanoseconds / 1e9
            if elapsed0 <= self.p['input_wait']:
                self.log.warn(f'입력 대기 {elapsed0:.1f}/{self.p["input_wait"]}s: '
                              + ('TF ' if x is None else '')
                              + ' '.join(n for n, _, _ in bad),
                              throttle_duration_sec=1.0)
                return
            if x is None:
                return self._stop(
                    'localization_lost',
                    f"TF {self.p['global_frame']}->{self.p['base_frame']} 없음 또는 "
                    f"{self.p['tf_timeout']}s 초과")
            # costmap stale. detail 에 bad_inputs **전부**와 그 tick 의 tf_age 를 같이
            # 싣는다. 하나만 실으면 사후에 'TF 도 같이 죽었는가'를 가릴 수 없다 --
            # 무선 링크가 끊기면 둘은 함께 죽는다 (실측 3/3).
            detail = '; '.join(
                [f'{n} costmap {topic} '
                 + ('수신 이력 없음' if age is None else f'{age:.1f}s 경과')
                 for n, topic, age in bad]
                + [f'tf_age {self._tf_age:.2f}s'])
            # 그 tf_age 가 tf_stale_hint 를 넘었으면 costmap 문제가 아니라 위치 상실로
            # 보고한다 (tf_stale_hint 주석). x is not None 이므로 _pose() 가 방금
            # _tf_age 를 갱신했고, tf_timeout 미달이라 여기까지 온 것이다.
            hint = self.p['tf_stale_hint']
            if hint > 0.0 and self._tf_age > hint:
                return self._stop('localization_lost', detail)
            return self._stop('internal', detail)
        with self.lock:
            self._pose_xy_th = x
        t1 = time.perf_counter()

        d_goal = float(np.linalg.norm(x[:2] - g['xy']))
        dth = abs(wrap(x[2] - g['yaw']))
        # Position first, heading second. A drive goal carries a yaw the drive
        # weights are too weak to land (w_head = 0.5 must not beat the goal
        # term), so on reaching xy we switch to the rotation weight set and
        # finish the heading in place - the same mechanism as an observation
        # turn, rather than a second controller.
        if d_goal < g['xy_tol']:
            if dth < g['yaw_tol']:
                return self._stop('reached')
            if self.phase == 'driving':
                self.log.info(f'xy reached, turning to {g["yaw"]:+.2f} rad (off by {dth:.2f})')
                self._set_mode('rotating')

        now = self.node.get_clock().now()
        elapsed = (now - g['t0']).nanoseconds / 1e9
        if elapsed > g['timeout']:
            return self._stop('timeout')

        self.mppi.grid = self.grid
        if d_goal < self.p['goal_switch_dist'] or self.phase == 'rotating':
            wp, blocked, wp_yaw = g['xy'], None, None
        else:
            wp, blocked, wp_yaw = self._carrot(x, g)
        if blocked is not None:
            # carrot_skip_max 안에 '자유 + 같은 성분'인 경로점이 없다.
            g['carrot_block_t'] = g['carrot_block_t'] or elapsed
            if elapsed - g['carrot_block_t'] > self.p['blocked_time']:
                a, b = blocked
                return self._stop('blocked', f'경로 ({a[0]:.2f}, {a[1]:.2f})~({b[0]:.2f}, '
                                             f'{b[1]:.2f}) 구간이 막혔고 우회로가 없다')
            # 판정이 설 때까지는 목표 방향을 유지한다. 접선도 목표 yaw 로 떨어뜨린다
            # -- 건너뛸 경로점이 없는 상황에서 '경로 접선'은 뜻이 없다.
            wp, wp_yaw = g['path'][-1], None
        else:
            g['carrot_block_t'] = None

        # heading 기준. 'goal' 이면 yaw_ref == g['yaw'] 가 항상 성립하므로 이전과 동일하다.
        yaw_ref = (wp_yaw if self.p['heading_ref'] == 'path' and wp_yaw is not None
                   else g['yaw'])
        # 비용 조회는 step() **앞**에서 한다. 뒤에서 하면 costmap 콜백이
        # (ReentrantCallbackGroup) 끼어들어 MPPI 가 본 것보다 새 grid 에서 읽힌다
        # -- 창이 mppi_ms p50 4.7 ms, 두 costmap 합계 ~4.3 Hz 라 tick 당 약 2 % 다.
        wpa = np.asarray(wp, float)
        seg = x[:2] + np.linspace(0.0, 1.0, 20)[:, None] * (wpa - x[:2])
        c_robot = float(self.grid.lookup(x[None, :2])[0])
        c_wp = float(self.grid.lookup(wpa[None])[0])
        c_seg = float(self.grid.lookup(seg).max())

        self._diag = None
        # 목표 셀 기준 단측 벌점. 목표가 장애물 가까이 있으면 그 셀의 비용이 목표
        # 유인을 이겨 MPPI 가 목표에 **들어가지 않는 것이 최적**이 된다. 실측
        # (2026-10-07, 목표 (9.0, 4.3), 벽에서 0.50 m, inflation 1.05/6.67):
        #   목표 셀 비용 0.418 -> 장애물 항 8.4/step,  목표 항 15*0.23 = 3.45/step
        #   -> 로봇이 목표 0.23 m 앞에서 10 s 얼어붙고 no_progress 로 끝났다.
        #
        # goal_switch_dist(D) 안에서 기준선을 0 -> c_goal 로 올린다. 계단으로 올리면
        # 전환 지점에서 비용이 튀고 그 불연속이 warm start 를 타고 지평선 안으로 밀려
        # 들어오므로, 거리에 따라 램프한다.
        #
        # **램프 모양이 임계 조건을 정한다.** 선형(ref = c_goal (1 - d/D))이면 목표
        # 셀의 잔여 벌점이 w_obs c_goal d/D 이고 목표 유인이 w_goal d 라, 둘 다 d 에
        # 비례해서 d 가 약분된다:
        #       목표가 이긴다  <=>  c_goal < w_goal * D / w_obs = 15*0.75/20 = 0.5625
        # 즉 거리와 무관하게 '되거나 안 되거나'이고, c_goal 0.5625 (= 벽에서 0.457 m)
        # 보다 가까운 목표는 상한과 무관하게 여전히 얼어붙는다. ANC_* 와 달리 이것은
        # 튜닝값이 아니라 두 가중치의 비에서 나오는 값이다.
        # 그래서 램프는 D/2 에서 c_goal 에 **완전히 도달**하게 둔다 -- d < D/2 에서
        # 잔여 벌점이 정확히 0 이 되어 목표 유인이 무조건 이긴다.
        # 상한 1.0 은 inflation 띠의 최댓값이다: lethal 셀은 할인하지 않는다.
        c_goal = min(float(self.grid.lookup(g['xy'][None])[0]), 1.0)
        D = self.p['goal_switch_dist']
        obs_ref = c_goal * min(1.0, max(0.0, 2.0 * (1.0 - d_goal / D)))

        anc = self._ancillary(x, g, wpa, yaw_ref) if self.p['biased'] else None
        u = self.mppi.step(x, wp, goal_theta=yaw_ref, cost_volume=None,
                           ancillary=anc, obs_ref=obs_ref)
        t2 = time.perf_counter()
        mp = self.mppi
        if mp.last_terms is not None:
            bi, tm = mp.last_best, mp.last_terms
            # 그룹 칸. biased off (또는 _ancillary 가 None) 이면 w_gauss 만 채운다.
            grp = {f'{q}_{n}': '' for q in ('w', 'is', 'hit')
                   for n in ('path', 'sneg', 'spos')}
            grp['w_gauss'] = round(mp.last_w_groups[0], 3)
            grp['is_gauss'] = round(mp.last_is_groups[0], 1)
            grp['hit_gauss'] = round(mp.last_hit_groups[0], 3)
            for q, n in enumerate(('path', 'sneg', 'spos'), start=1):
                if q < len(mp.last_w_groups):
                    grp[f'w_{n}'] = round(mp.last_w_groups[q], 3)
                    grp[f'is_{n}'] = round(mp.last_is_groups[q], 1)
                    grp[f'hit_{n}'] = round(mp.last_hit_groups[q], 3)
            self._diag = dict(
                eta=round(mp.last_eta, 1), lam=round(float(mp.lam), 4),
                n_hit=mp.last_n_hit, fallback=int(mp.last_fallback),
                c_robot=round(c_robot, 3), c_wp=round(c_wp, 3), c_seg_max=round(c_seg, 3),
                lam_hit=int(mp.last_lam_hit),
                S_min=round(mp.last_S[0], 1), S_p50=round(mp.last_S[1], 1),
                S_max=round(mp.last_S[2], 1),
                best_goal=round(float(tm['goal'][bi]), 1),
                best_obs=round(float(tm['obs'][bi]), 1),
                best_head=round(float(tm['head'][bi]), 1),
                best_ctrl=round(float(tm['ctrl'][bi]), 1),
                w_fwd=round(mp.last_w_bins[0], 3), w_left=round(mp.last_w_bins[1], 3),
                w_back=round(mp.last_w_bins[2], 3), w_right=round(mp.last_w_bins[3], 3),
                yaw_err=round(wrap(x[2] - yaw_ref), 4), yaw_ref=round(yaw_ref, 4),
                obs_ref=round(obs_ref, 4), **grp)

        # 최종 접근 ω 캡. rotating 에서만 건다 -- 그때만 yaw_ref == g['yaw'] 이고
        # (wp_yaw 가 None 이므로) 남은 일이 'yaw 를 착지시키는 것' 하나다. driving 에
        # 걸면 안 된다: heading_ref='path' 에서 |yaw_err| 가 작은 것은 '거의 다 왔다'가
        # 아니라 **접선 추종이 잘 되고 있다**는 뜻이고 (실측 p50 7.8 deg), 거기서 ω 를
        # 0.20 으로 묶으면 0.4 m/s 에서 최소 선회반경이 2.0 m 가 되어 코너를 못 돈다.
        # xy 도달 후 heading 만 남은 drive goal 도 _set_mode('rotating') 를 지나므로
        # 이 조건 하나로 두 경우를 다 덮는다.
        #
        # u 는 self.U[0].copy() (또는 _sgf 경로의 새 배열)라 내부 상태와 별칭이 아니므로
        # 여기서 깎아도 warm start 는 깨지지 않는다. 다만 MPPI 는 캡을 **모른다**:
        # u_prev / u_hist 에는 깎기 전 값이 남고, 다음 tick 에도 같은 ω 를 제안한다.
        # 상태는 매 tick TF 에서 다시 잡으므로 수렴은 하고, 깎는 방향이 항상 0 쪽이라
        # 오버슈트로 갈 수 없다 -- 느려질 뿐이다. rd_w (ω rate 비용)를 켜는 날에는
        # u_prev 가 실제 지령과 어긋나므로 그때 같이 봐야 한다. 지금은 rd_w = 0 이다.
        # v_max_wz 를 런타임에 낮춰 MPPI 가 스스로 지키게 하는 쪽이 깔끔하지만
        # mppi.py 를 건드린다.
        wa, wc = self.p['yaw_approach'], self.p['w_approach']
        if (self.phase == 'rotating' and wa > 0.0 and wc > 0.0
                and abs(wrap(x[2] - yaw_ref)) < wa):
            u[2] = float(np.clip(u[2], -wc, wc))
        with self.lock:
            self._u = u

        # BLOCKED: every rollout ends inside the lethal band, for long enough
        # that it is the world and not one bad draw.
        if self.mppi.last_fallback:
            g['blocked_t'] = g['blocked_t'] or elapsed
            if elapsed - g['blocked_t'] > self.p['blocked_time']:
                return self._stop('blocked')
        else:
            g['blocked_t'] = None

        # NO_PROGRESS: the distance left has not dropped by no_progress_distance
        # within no_progress_time. Measured on the path, not on the straight
        # line, so rounding a corner still counts as progress.
        rem = self._remaining(x[:2], g['path'])
        if g['prog_d'] is None:
            g['prog_d'], g['prog_t'] = rem, elapsed
        else:
            # 요구 거리는 **창 시작 시 남은 거리의 절반**을 넘지 않는다. 고정 0.5 m 는
            # 목표 0.5 m 안에서 원리적으로 만족될 수 없다 -- 남은 거리가 그보다 작으니
            # 아무리 잘 가도 0.5 m 를 줄일 수 없고, 최종 접근이 no_progress_time 을
            # 넘기면 무조건 죽는다. 시뮬에서 목표 0.23 m 앞에 멈춘 run 이 그렇게 끝났다.
            # (실기 30 회의 no_progress 9 건은 전부 rem >= 1.45 m 에서 끝났으므로 그쪽
            #  실패의 원인은 아니다 -- 별개의 死구간이다.)
            # 절반씩 줄어 수렴한다: rem 0.23 -> need 0.115 -> 그 거리는 xy_tolerance
            # 0.15 안이라 reached 가 먼저 난다.
            need = min(self.robot['no_progress_distance'], 0.5 * g['prog_d'])
            if rem < g['prog_d'] - need:
                g['prog_d'], g['prog_t'] = rem, elapsed
            elif (elapsed - g['prog_t'] > self.robot['no_progress_time']
                  and not self._waiting_for_dynamic()):
                return self._stop('no_progress')

        if self.p['viz']:
            self._draw()
        if self._csv is not None:
            # 메모리에 쌓지 않고 바로 쓴다. 180 s 짜리 goal 이면 1800 행이고,
            # 아무도 안 읽는 리스트로 두었던 것이 이전 구현의 실수였다.
            v = self._velocity()
            self._csv.writerow(dict(
                t=round(elapsed, 4), phase=self.phase,
                x=round(x[0], 4), y=round(x[1], 4), th=round(x[2], 4),
                wp_x=round(float(wp[0]), 4), wp_y=round(float(wp[1]), 4),
                near_i=self._nearest(x[:2], g['path']),
                ux=round(u[0], 4), uy=round(u[1], 4), uw=round(u[2], 4),
                v_meas='' if v is None else round(v[0], 4),
                w_meas='' if v is None else round(v[1], 4),
                rem=round(rem, 3),
                cm_ms=round(1e3 * (t1 - t0), 3), mppi_ms=round(1e3 * (t2 - t1), 3),
                heading_ref=self.p['heading_ref'],
                w_odom='' if self._w_odom is None else round(self._w_odom, 4),
                tf_age=round(self._tf_age, 3),
                # solve 가 안 돈 tick 이면 전부 빈 칸이다 (DIAG_FIELDS 주석 참조).
                **(self._diag or EMPTY_DIAG)))
            self._csv_f.flush()

    def _publish(self):
        """30 Hz. Rotates the held global-frame command into the base frame with
        a FRESH heading: between solves the base keeps turning, and a command
        rotated once at solve time would steer into the turn."""
        state = self.state
        with self.lock:
            u = self._u.copy() if state == RUNNING else self._cmd.copy()

        if state == STOPPING:
            # Ramp, do not snap. The base is a velocity source, so the ramp is
            # the deceleration the spec asks for. 병진과 회전은 **다른 한계**를 쓴다:
            # 공용 step 이면 회전이 1.0 rad/s^2 로 램프해 ω^2/(2*1.0) = 3.33 deg 를
            # 쓸어내고, 탐색 회전의 +-5.3 deg 예산을 혼자 다 먹는다 (decel_limit_w 주석).
            step = self.p['decel_limit'] / self.p['publish_frequency']
            step_w = (self.p['decel_limit_w'] or self.p['decel_limit']) \
                / self.p['publish_frequency']
            n = float(np.linalg.norm(u[:2]))
            u[:2] = u[:2] * (max(0.0, n - step) / n) if n > 1e-9 else 0.0
            u[2] = math.copysign(max(0.0, abs(u[2]) - step_w), u[2])
            self._check_stopped()
        elif state == IDLE:
            u[:] = 0.0

        with self.lock:
            self._cmd = u
            if np.any(np.abs(u) > 1e-9):
                self._moved = True
        msg = Twist()
        # Fresh read, not the one the solve cached 100 ms ago: at 0.5 m/s that
        # pose is already 5 cm stale, and the heading it carries is what the
        # whole rotation is for. No TF -> publish a zero, never hold the last
        # command: VelocityControl keeps applying whatever it was last given.
        if state != IDLE:
            self._vel_sample()
        x = self._pose() if state != IDLE else None
        if x is not None:
            c, s = math.cos(-x[2]), math.sin(-x[2])
            msg.linear.x = float(c * u[0] - s * u[1])
            msg.linear.y = float(s * u[0] + c * u[1])
            msg.angular.z = float(u[2])
        self.cmd_pub.publish(msg)
        if state != RUNNING:
            self._log_row(u, x)

    def _log_row(self, u, x):
        """STOPPING tick 한 행. _solve 는 state != RUNNING 이면 첫 줄에서 리턴하므로
        여기서 쓰지 않으면 정지 구간이 로그에 **전혀** 남지 않는다. 그러면 정지 후
        실제 yaw, stopped=true 까지 걸린 시간, 정지 상태의 속도 잡음 바닥 -- 회전
        정확도를 약속하는 데 필요한 세 값 전부를 측정할 수 없다.

        RUNNING 중에는 _solve 가 쓰므로 여기서 쓰지 않는다. 둘 다 같은 10 Hz 라
        그러지 않으면 모든 행이 두 번 들어간다.

        MPPI 진단 열은 EMPTY_DIAG 로 비운다 -- solve 가 돌지 않은 tick 이고, 그
        계약(DIAG_FIELDS 주석)에 yaw_err/yaw_ref 도 들어간다. 그래서 정착 yaw 오차는
        여기서 쓰지 않고 분석에서 만든다: 이 행의 th 와, solve 가 마지막으로 남긴
        행의 yaw_ref 의 차다 (rotating phase 에서는 yaw_ref == goal 의 yaw 다).

        _check_stopped() 가 이 tick 에 _finish() 를 불렀다면 로그가 이미 닫혀 있어
        마지막 한 행(0.1 s)은 빠진다. 그 시점 ω 는 이미 0 이라 yaw 는 변하지 않는다.
        """
        if self._csv is None:
            return
        with self.lock:
            g = self._goal
        if g is None:
            return
        v = self._velocity()
        self._csv.writerow(dict(
            t=round((self.node.get_clock().now() - g['t0']).nanoseconds / 1e9, 4),
            phase=self.phase,
            x='' if x is None else round(x[0], 4),
            y='' if x is None else round(x[1], 4),
            th='' if x is None else round(x[2], 4),
            ux=round(u[0], 4), uy=round(u[1], 4), uw=round(u[2], 4),
            v_meas='' if v is None else round(v[0], 4),
            w_meas='' if v is None else round(v[1], 4),
            heading_ref=self.p['heading_ref'],
            tf_age=round(self._tf_age, 3),
            **EMPTY_DIAG))
        self._csv_f.flush()

    def _check_stopped(self):
        """속도는 TF 차분에서만 온다. v is None 은 '모른다'이고, 모르면 멈췄다고
        하지 않는다 -- 위치를 잃은 채 관성으로 굴러가는 로봇을 정지로 보고하는 것이
        이 계약에서 가장 나쁜 거짓말이다."""
        # 측정 없이 '멈췄다'고 말할 수 있는 유일한 경우: 직전 goal 이 **측정으로**
        # stopped=true 로 끝났고, 그 뒤로 0 이 아닌 명령을 한 번도 내지 않았을 때.
        # 그때는 베이스가 움직일 이유가 없었다. 추측이 아니라 다른 종류의 근거이고,
        # 위치를 잃어 속도를 못 재는 상황에서도 성립한다.
        #
        # 노드 기동 후 첫 goal 이거나 직전이 stopped=false 였으면 쓰지 않는다 --
        # 그때는 베이스 상태를 아예 모른다.
        #
        # 한계: 외부 조작(게임패드, 손으로 민 경우)으로 움직였다면 감지할 수 없다.
        # 우리 명령만 보기 때문이다.
        if self._rest_confirmed and not self._moved:
            self._finish(True)
            return
        v = self._velocity()
        now = time.monotonic()
        moving = (v is None or v[0] > self.p['stop_speed']
                  or v[1] > self.p['stop_yaw_rate'])
        if moving:
            self._below_t = None
        elif self._below_t is None:
            self._below_t = now
        stopped = self._below_t is not None and now - self._below_t >= self.p['stop_hold']
        if not stopped and now - self._stop_t0 < self.p['stop_timeout']:
            return
        if not stopped:
            # Reported honestly rather than hidden: the contract is that result
            # carries whether the robot really came to rest.
            self.log.error(f'still moving after {self.p["stop_timeout"]}s; '
                           f'reporting stopped=False (v={v})')
        # 측정 결과만 이 플래그를 갱신한다. 위의 '움직인 적 없음' 경로는 이미
        # True 인 것을 소비할 뿐 새로 세우지 않는다.
        self._rest_confirmed = bool(stopped)
        self._finish(stopped)

    def _finish(self, stopped):
        # 순서 계약: _done 과 IDLE 은 반드시 같은 락 안에서 함께 바뀐다. take_outcome()
        # 으로 결과를 읽은 프론트엔드는 곧바로 result 를 보내고, 그 result 를 받은
        # 클라이언트는 즉시 다음 goal 을 던진다. IDLE 전환이 한 틱이라도 늦으면 그
        # goal 이 busy 로 거절된다 -- 연속 요청(관측 회전 4연발)이 깨지는 방식이다.
        with self.lock:
            self._done = (self._outcome, self._pose_xy_th, bool(stopped), self._detail)
            self.state, self.phase = IDLE, 'idle'
            self._u[:] = 0.0
            self._goal = None
        self._close_log()

    ANC_K_YAW = 1.0              # rad/s per rad. 포화는 _limit 이 맡는다

    @staticmethod
    def _at_arc(path, arc, s):
        """호 길이 s 지점의 경로 좌표 (선형 보간)."""
        s = min(max(s, 0.0), float(arc[-1]))
        k = min(max(int(np.searchsorted(arc, s)), 1), len(path) - 1)
        f = (s - arc[k - 1]) / max(float(arc[k] - arc[k - 1]), 1e-9)
        return path[k - 1] + f * (path[k] - path[k - 1])

    def _ancillary(self, x, g, wp, yaw_ref):
        """Biased-MPPI 보조 제어열 (3, T, 3): 경로 추종, side-, side+. 또는 None.

        세 열 모두 **같은 yaw 법칙**을 쓴다: w = clip(k * wrap(yaw_ref - th)).
        yaw_ref 는 이 tick 에 step() 으로 넘어가는 그 스칼라다. 비용의 head 항이
        |wrap(theta - yaw_ref)| 를 그 값으로 매기므로, 다른 자세를 전제한 열은 비용상
        체계적으로 불리해진다 -- 측면 기동에 w = 0 을 주면 heading_ref=path 에서 side 가
        head 항 때문에 늘 지는 것이 그 예다. 경로 추종 열에도 t 마다 다른 접선이 아니라
        같은 스칼라를 쓴다. 같은 이유다: 비용은 지평선 전 구간에 goal_theta 하나를 쓴다.
        """
        T, dt = self.mppi.T, self.mppi.dt
        v_nom, w_max = float(self.mppi.v_max[0]), float(self.mppi.v_max[2])
        path = g['path']
        wp = np.asarray(wp, float)

        # 제자리 회전 중에는 병진 후보를 내지 않는다. rotating 단계의 목표는 위치를
        # 지키며 yaw 만 돌리는 것이고(rot_w_goal 60 이 위치를 잡는다), 전진/측면 열은
        # 그 목표와 정면으로 어긋난다.
        if self.phase == 'rotating':
            return None

        e_f = wp - x[:2]
        n = float(np.linalg.norm(e_f))
        if n < 1e-6 or len(path) < 2:
            # 캐럿이 로봇 위에 있거나 경로가 한 점이다 -- 측면 기동의 기준 방향이
            # 정의되지 않는다. 가우시안만으로 돈다 (그 tick 의 그룹 칸은 빈다).
            return None
        e_f /= n
        e_s = np.array([-e_f[1], e_f[0]])                 # 좌 = side+
        out = np.zeros((3, T, 3))

        def omega(th):
            return float(np.clip(self.ANC_K_YAW * wrap(yaw_ref - th), -w_max, w_max))

        # (a) 경로 추종. **호 길이를 따라 전진한다.**
        #
        #     lookahead 점으로 직선 진행하면 매 스텝 예측 위치에서 lookahead 를 다시
        #     잡아 경로의 곡률을 가로지르고 **코너를 깎는다**. 실측 (clutter 정체 시점,
        #     라이브 max(static, local) grid): 경로 자체의 최대 비용 0.500, 로봇->캐럿
        #     직선은 lethal 0/40 인데 **롤아웃만 lethal 7/25** 로 기둥 0.30 m 안에
        #     들어갔다 (inscribed 0.37). 그래서 ancillary 21 개가 전원 탈락해
        #     Sum(w) = 0.000 이 되고 biasing 이 아무 효과도 내지 못했다 -- 그때
        #     n_hit 이 2 -> 65 (= 블록 크기 63) 로 뛴 것이 유일한 단서였다.
        #     호 길이로 전진하면 롤아웃이 구성상 경로 위에 놓인다
        #     (수정 후 lethal 0/25, 지령 0.010 -> 0.372 m/s).
        #
        #     목표 근처에서 감속한다: v = min(v_nom, |goal - p| / t_settle).
        #     t_settle = T * dt = 2.5 s, 즉 **지평선 그 자체**다. 새 상수를 만들지 않는
        #     유일한 선택이고, 뜻이 명확하다 -- "지평선이 끝날 때 목표에 닿는 속도".
        #     이게 없으면 열이 목표를 지나쳐 계속 달리고, 지평선의 남은 구간이 목표를
        #     넘어선 궤적을 평가하는 데 쓰인다.
        t_settle = T * dt
        arc = np.concatenate([[0.0],
                              np.linalg.norm(np.diff(path, axis=0), axis=1).cumsum()])
        p, th = x[:2].astype(float).copy(), float(x[2])
        s_arc = float(arc[self._nearest(p, path)])
        for t in range(T):
            v_cap = min(v_nom, float(np.linalg.norm(g['xy'] - p)) / t_settle)
            s_arc = min(s_arc + v_cap * dt, float(arc[-1]))
            d = self._at_arc(path, arc, s_arc) - p
            nd = float(np.linalg.norm(d))
            # 로봇이 경로에서 벗어나 있으면 d 가 한 스텝 거리보다 멀다. 그때는
            # 방향만 쓰고, 가까우면 d/dt 로 그 점에 정확히 올려놓는다.
            v = d / dt if nd <= v_cap * dt * 1.5 else v_cap * d / nd
            n = float(np.linalg.norm(v))
            if n > v_nom:
                v = v * (v_nom / n)        # 벡터 스케일. v = v_nom 은 스칼라 대입 버그다
            w = omega(th)
            out[0, t] = (v[0], v[1], w)
            p = p + v * dt
            th += w * dt

        # (b)(c) side-, side+. 캐럿 방향 기준 전진 g == 1, 횡 +- h(t).
        # (1, +-h) 의 노름이 최대 sqrt(2) 라 _limit 이 디스크로 투영한다 -> 완전 램프에서
        # 전진 성분이 v_max/sqrt(2) = 0.71 v_max 로 줄어든다. **의도한 것이다**: 옆으로
        # 크게 비키면 전진이 느려지는 것이 맞다. 그래서 정규화하지 않는다.
        h = np.minimum(1.0, np.arange(T) / max(1.0, self.p['anc_ramp_frac'] * (T - 1)))
        for row, sign in ((1, -1.0), (2, +1.0)):
            th = float(x[2])
            for t in range(T):
                v = v_nom * (e_f + sign * h[t] * e_s)
                w = omega(th)
                out[row, t] = (v[0], v[1], w)
                th += w * dt
        return out

    def _tangent(self, path, k):
        """path[k] 에서의 경로 접선 yaw, 앞이 없으면 None.

        한 세그먼트(= 격자 0.05 m)로 재면 A* 경로의 계단 때문에 대각선 구간에서
        0 도와 90 도를 번갈아 낸다. tangent_span 앞의 점까지의 **현**으로 재서 그
        계단을 평균한다 -- 필터보다 싸고, '경로가 실제로 꺾인 양 이상으로는 절대
        튀지 않는다'는 성질이 공짜로 따라온다.

        진짜 90 도 코너에서는 기준이 로봇보다 앞서 돌지만(0.30 m 주행에 90 도 =
        1.05 rad/s > v_max_wz) w_head 는 서보가 아니라 약한 당김이라 해가 없다.
        그래서 rate limit 은 넣지 않았다. 남는 위험은 _nearest 가 뒤로 튈 때
        (실측 run 당 최대 20 회) yaw_ref 도 함께 움직이는 것인데, 미리 기계를 만들지
        않고 yaw_ref 를 CSV 에 찍어 측정한 뒤 판단한다.
        """
        d = np.linalg.norm(np.diff(path[k:], axis=0), axis=1).cumsum()
        if len(d) == 0:
            return None                      # 경로 끝 -- 호출부가 goal yaw 로 떨어진다
        j = min(k + 1 + int(np.searchsorted(d, self.p['tangent_span'])), len(path) - 1)
        v = path[j] - path[k]
        n = float(np.linalg.norm(v))
        return None if n < 1e-6 else float(np.arctan2(v[1], v[0]))

    def _carrot(self, x, g):
        """경로 위에서 다음 목표점을 고른다. (carrot, 막힌 구간 or None, 접선 yaw or None)

        기존 lookahead 지점부터 경로를 전진하며, **자유이면서 로봇과 같은 연결 성분**인
        첫 점을 고른다. 두 조건이 모두 필요하다:

          자유     : 합성 cost(= max(static, local)) < LETHAL. 지도에 없는 장애물도 걸린다.
          연결성   : local 창의 8-연결 성분이 로봇 셀과 같아야 한다. 이게 없으면 벽
                     **너머**의 자유 경로점을 집어 로봇이 벽으로 밀고 들어간다 --
                     상자(돌아갈 수 있음)와 벽(못 감)은 경로상 차단 구간 길이가
                     1.1 m 대 0.94 m 로 비슷해서 거리로는 구분되지 않는다.

        창 밖 후보는 창 경계까지의 도달성만 본다 (그 앞의 마지막 창 안 경로점의 성분).
        한계: 창 밖으로 크게 휘어 돌아오는 우회는 '막힘'으로 판정될 수 있다.
        """
        path, res = g['path'], self.p['carrot_skip_max']
        i = self._nearest(x[:2], path)
        if len(path) < 2:
            return path[-1], None, None
        seg = np.linalg.norm(np.diff(path[i:], axis=0), axis=1).cumsum()
        j0 = min(i + 1 + int(np.searchsorted(seg, self.p['lookahead_dist'])), len(path) - 1)

        tail = path[j0:]
        cost = self.grid.lookup(tail)
        free = cost < CostmapGrid.LETHAL
        free[-1] = True                      # goal 은 cost 로 건너뛰지 않는다 (규칙 3)

        local = self.grid.local
        lab = local.labels()
        r, c, ins = local.cell(x[:2][None])
        mine = int(lab[r[0], c[0]]) if ins[0] else 0
        # 로봇 셀 자체가 자유가 아니면(inflation 띠를 스치는 중) 성분을 말할 수 없다.
        # 그때는 연결성 검사를 건너뛰고 cost 만 본다 -- 안 그러면 띠에 닿을 때마다
        # 모든 후보가 탈락해 BLOCKED 가 난다.
        if mine == 0:
            self.log.warn('로봇 셀이 자유 성분이 아니다: 연결성 검사 생략',
                          throttle_duration_sec=5.0)

        tr, tc, tins = local.cell(tail)
        dist = np.concatenate([[0.0], np.linalg.norm(np.diff(tail, axis=0), axis=1).cumsum()])
        last_in = None                        # 직전까지의 마지막 '창 안' 라벨
        for k in range(len(tail)):
            if dist[k] > res:
                break
            if tins[k]:
                last_in = int(lab[tr[k], tc[k]])
                ok_conn = mine == 0 or last_in == mine
            else:
                # 창 밖: 경계까지 닿을 수 있으면 채택한다.
                ok_conn = mine == 0 or last_in == mine
            if free[k] and ok_conn:
                # tail = path[j0:] 이므로 tail[k] 는 path[j0 + k] 다.
                return tail[k], None, self._tangent(path, j0 + k)
        blocked_from = tail[0]
        blocked_to = tail[min(k, len(tail) - 1)]
        return None, (blocked_from, blocked_to), None

    def _draw(self):
        n = int(self.p['viz_samples'])
        if n <= 0 or not hasattr(self.mppi, 'last_X'):
            return
        stamp = self.node.get_clock().now().to_msg()

        def mk(i, kind, width, rgba):
            m = Marker()
            m.header.frame_id, m.header.stamp = self.p['global_frame'], stamp
            m.ns, m.id, m.type, m.action = 'mppi', i, kind, Marker.ADD
            m.scale.x = width
            m.pose.orientation.w = 1.0
            (m.color.r, m.color.g, m.color.b, m.color.a) = rgba
            return m

        def pt(xy):
            return Point(x=float(xy[0]), y=float(xy[1]), z=0.02)

        # One LINE_LIST for all samples beats n markers through the bus, and a
        # uniform slice rather than the top-weighted ones: the softmax can put
        # nearly all the weight on one rollout, which would draw one line n times.
        samples = mk(0, Marker.LINE_LIST, 0.004, (0.2, 0.6, 1.0, 0.25))
        for k in range(0, self.mppi.K, max(1, self.mppi.K // n)):
            xy = self.mppi.last_X[k, :, :2]
            for a, b in zip(xy, xy[1:]):
                samples.points.append(pt(a))
                samples.points.append(pt(b))
        best = mk(1, Marker.LINE_STRIP, 0.02, (1.0, 0.85, 0.1, 0.95))
        best.points.extend(pt(q) for q in self.mppi.last_nominal[:, :2])
        self.viz_pub.publish(MarkerArray(markers=[samples, best]))
