"""Local planner: MPPI over a pure-integrator holonomic base.

Notation follows Williams et al. (T-RO 2018): sample index k, time index t,
nominal input U, sampled input V. Algorithm unchanged from nav_mppi.py; the
constructor kwargs are gathered into a dataclass per the project convention.

State  x = (x, y, theta)   global frame
Action u = (vx, vy, omega) global frame -> rollout = cumsum, no discretisation error

One step() is a single sample -> evaluate -> reweight sweep:

    eps^k_t ~ N(0, Sigma), Sigma = diag(sigma^2)     K x T perturbations of U
    v^k_t   = limit(u_t + eps^k_t)                   disc on (vx, vy), box on omega
    eps^k_t := v^k_t - u_t                           re-derived after the limit

    x^k_{t+1} = x^k_t + v^k_t * dt                   Euler, and exact here -- see _rollout

    S^k = sum_t gamma^t q(x^k_{t+1}, t)              state cost, discounted; X holds x_1..x_T
        + is_coeff * sum_t u_t^T Sigma^-1 eps^k_t    importance sampling, NOT discounted
    S^k = +inf for any rollout that reads LETHAL     infeasible: zero weight

    lam solved this cycle so that eta lands in [eta_min, eta_max]   see _solve_lam
    w^k = exp(-(S^k - beta) / lam) / sum_j ...       beta = min_j over the survivors
    U  <- U + sum_k w^k eps^k                        == sum_k w^k v^k, since sum_k w^k = 1

    u = SG([the last 4 executed commands; U[0..4]])  the command, filtered
    U <- roll(U, -1); U[-1] = U[-2]                  warm start, unfiltered, tail held

Three things above are not in Williams et al., and each is here because it was
measured on the four ROS scenarios (4 x 5 episodes per row):

    eta held at 5-10% of K   the softmax saturates otherwise: eta = 1.0, and the
                             update becomes "execute one raw noise sample", which
                             reads as a shaking robot. jerk 8.74 -> 0.86.
    is_coeff fixed, not lam  with the IS term scaled by the temperature the two
                             cancel in the exponent, and eta cannot leave ~2 at
                             cruise however hot the softmax runs (measured: eta
                             1.01..1.75 over lam 0.5..1000). Decoupled, the term
                             is a fixed control penalty rather than an exact
                             importance correction - the deliberate trade.
    SG on the command only   the filter never touches the warm start, so the plan
                             stays the optimiser's. jerk 26.76 -> 8.74.

Their product is the whole reduction: 26.76 -> 0.86 against nav2_mppi's 0.42.

Stage cost q is charged at every horizon step and there is no terminal cost: wp
is a moving lookahead point, so the goal term is already present throughout.

    q(x, t) = w_goal * ||p - wp||                    p = x[:2], metres
            + w_obs  * c_static(p)                   grid.py, baked EDT field, [0, 10]
            + w_dyn  * c_dyn(t, p)                   cost_volume.py, [0, 1], optional
            + w_head * |wrap(theta - goal_theta)|    wrap -> [-pi, pi)
            + q_u(u_t, u_{t-1})                      control cost, see below

The control cost shapes the command sequence itself rather than the states it
produces. R and R_d are diagonal, so each is stored as a (3,) vector.

    q_u(u_t, u_{t-1}) = 1/2 u_t^T R u_t                      input magnitude
                      + 1/2 (u_t - u_{t-1})^T R_d (...)      input rate
                      + w_dir * c_dir(u_t, u_{t-1})          direction change

    c_dir = (1 - cos dpsi) * ||(vx_t, vy_t)||
    cos dpsi = (v_t . v_{t-1}) / (||v_t|| ||v_{t-1}||)       vx, vy only -- no omega

c_dir is scaled by speed on purpose: a heading flip while nearly stopped is cheap
(and ill-defined), one at full speed is not. u_{t-1} at t = 0 is the command the
previous solve executed, so the penalty spans solves instead of resetting each time.

Two terms of the paper's four-term weight are absent by construction:
(lambda/2) sum_t u^T Sigma^-1 u does not depend on k and cancels inside the
softmax, and (lambda (1 - 1/nu) / 2) sum_t eps^T Sigma^-1 eps has coefficient
zero at nu = 1 -- this planner never inflates the exploration covariance.
"""

from dataclasses import dataclass

import numpy as np
from scipy.signal import savgol_coeffs

# grid.bake()'s cap, and CostmapGrid's: the value of every cell inside the robot
# radius. A rollout that reads it at any step has put the base into an obstacle.
LETHAL = 10.0


@dataclass
class MPPIConfig:
    """Verified defaults (PLAN section 7): w_goal=5 fails the doorway, w_goal=15
    passes it. The obstacle `decay` PLAN pairs with this one is a Grid.bake()
    parameter, not a field here.

    Field names are a CLI surface: sweep.py builds overrides by field name
    (`python -m nav.sweep --mppi w_dyn=60,150`), so renaming one breaks both the
    sweep and the result keys saved beside it.
    """
    # Sampling. K=512 is enough because every solve warm-starts from the last one;
    # T*dt = 2.5 s is matched to the nav2 baseline's horizon so the U trap is the
    # same problem for both.
    K: int = 512
    T: int = 25
    dt: float = 0.1                                  # s, and the solve period: step() shifts U by one dt
    # Exploration std. 0.35 lets a forward rollout's lateral spread reach 0.175 m
    # by the end of the horizon - exactly the half-width of the 1.05 m pillar gap,
    # so most rollouts through it were dropped as infeasible and the survivors
    # averaged to a standstill. 0.15 keeps them inside. omega is left wide: the
    # goal heading has only w_head behind it and needs the room.
    sigma: tuple[float, float, float] = (0.15, 0.15, 0.60)
    # Actuator limit. (vx, vy) is held to the disc of radius v_max[0], not a box:
    # a powered caster rolls the same speed in every direction, and the box let
    # the planner ask for 0.707 m/s diagonally - 50-73% of its commands.
    v_max: tuple[float, float, float] = (0.5, 0.5, 1.0)
    # IS coefficient, deliberately NOT the softmax temperature (see step (6)).
    is_coeff: float = 0.5
    # Effective-sample band, 5-10% of K. Below it the update picks one rollout and
    # the base shakes; well above it rollouts that disagree cancel and the base
    # will not commit through a narrow gap (clutter 0/5 at this setting today).
    eta_min: float = 25.6
    eta_max: float = 51.2
    w_goal: float = 15.0                             # 5면 문틈 통과 실패, 15 이상이면 통과
    w_obs: float = 20.0                              # c_static in [0,10] -> 스텝당 최대 200, 가장 무겁다
    w_dyn: float = 60.0                              # 30이면 충돌률 0.20, 60이면 0.00, 120은 freezing
    w_head: float = 0.5                              # 스텝당 최대 ~1.57. 주행 항을 이기지 못하도록 작게
    gamma: float = 0.98                              # 상태비용 할인. T=25 끝단 가중치 0.98^24 ~ 0.61
    # Control cost q_u. R = diag(r_v, r_v, r_w), R_d = diag(rd_v, rd_v, rd_w) --
    # 병진/회전은 단위가 달라 따로 둔다. 전부 스칼라라 sweep.py가 그대로 훑을 수 있다.
    # Swept on the ROS scenarios and left at zero: rd_w cuts omega sign flips
    # (48.8% -> 37.6% at 2) but from 10 the base wanders and touches walls, and
    # rd_v moved translational jerk by less than one standard error at 1, 3 and 10.
    # The reason is structural - only u_0 - u_prev ever reaches the robot, and it
    # is one of the 25 increments the rate cost scores.
    r_v: float = 0.0                                 # 입력 크기, vx/vy
    r_w: float = 0.0                                 # 입력 크기, omega
    rd_v: float = 0.0                                # 입력 변화율, vx/vy
    rd_w: float = 0.0                                # 입력 변화율, omega
    w_dir: float = 0.20                              # 방향 전환. c_dir in [0, 2*||v||] -> 스텝당 최대 ~1.4
    # Savitzky-Golay over [the last (window-1)/2 executed commands; U[0..]], centre
    # value out. 0 disables it. The window must be odd, > sgf_order and <= T; the
    # constructor legalises it.
    sgf_window: int = 9
    sgf_order: int = 2
    # Biased-MPPI (arXiv:2401.09241). ancillary 제어열 자체는 step() 이 인자로 받는다 --
    # 이 둘은 '받은 것을 어떻게 샘플에 섞을지'만 정한다. 생성은 경로를 아는 쪽(제어 노드)
    # 의 일이고, MPPI 는 끝까지 경로를 모른다.
    #
    # anc_copies 사본 중 **첫 하나는 노이즈가 없다** -- 의도한 기동이 항상 후보에
    # 올라가야 한다. 나머지는 anc_sigma_frac * sigma 로 흔든다. 작게 두는 이유: 의도한
    # 기동을 지우지 않고 그 주변만 보려는 것이다. 0.25 면 (0.0375, 0.0375, 0.075).
    anc_copies: int = 21
    anc_sigma_frac: float = 0.25
    seed: int = 0                                    # 인스턴스 로컬 RNG -- 전역 시드는 건드리지 않는다


class MPPI:
    def __init__(self, grid, cfg: MPPIConfig | None = None):
        cfg = cfg or MPPIConfig()
        self.cfg, self.grid = cfg, grid
        # cfg 필드를 평평하게 펴 둔다: 핫 루프의 속성 조회를 줄이고, 테스트가
        # mppi.dt / mppi.T / mppi.v_max를 직접 읽는다 (tests/test_mppi.py).
        self.K, self.T, self.dt = cfg.K, cfg.T, cfg.dt
        self.sigma = np.asarray(cfg.sigma, float)
        self.v_max = np.asarray(cfg.v_max, float)
        assert cfg.v_max[0] == cfg.v_max[1], f'the disc needs one translational limit, got {cfg.v_max}'
        self.w_goal, self.w_obs, self.w_dyn, self.w_head = cfg.w_goal, cfg.w_obs, cfg.w_dyn, cfg.w_head
        # 대각행렬이라 (3,) 벡터로 들고 있는다: u^T R u == sum_d R_d u_d^2.
        self.R = np.array([cfg.r_v, cfg.r_v, cfg.r_w], float)
        self.R_d = np.array([cfg.rd_v, cfg.rd_v, cfg.rd_w], float)
        self.w_dir, self.is_coeff = cfg.w_dir, cfg.is_coeff
        self.eta_min, self.eta_max = cfg.eta_min, cfg.eta_max
        self.anc_copies = cfg.anc_copies
        self.anc_sigma = cfg.anc_sigma_frac * self.sigma
        assert 0 < cfg.eta_min <= cfg.eta_max <= cfg.K, f'eta band {cfg.eta_min}..{cfg.eta_max} of K={cfg.K}'

        # savgol는 홀수 창에 window > order를, 그리고 window <= T를 요구한다.
        self.sgf_window = min(cfg.sgf_window, cfg.T - (1 - cfg.T % 2)) | 1 if cfg.sgf_window else 0
        self.sgf_order = cfg.sgf_order
        if self.sgf_window <= self.sgf_order:
            self.sgf_window = 0                      # 창이 차수보다 좁으면 필터가 성립하지 않는다
        self._sgf_c = savgol_coeffs(self.sgf_window, self.sgf_order) if self.sgf_window else None

        self.disc = cfg.gamma ** np.arange(cfg.T)    # (T,) 스텝별 할인 -- 상태비용에만 걸린다
        self.U = np.zeros((cfg.T, 3))                # 명목 입력열. step() 사이에 남는 이것이 warm start다
        self.u_prev = np.zeros(3)                    # 직전에 '실행한' 명령 = 지평선 t=0에서의 u_{t-1}
        # 출력 SG의 과거 절반, 오래된 것부터. 정지 상태에서 출발하므로 0으로 시작한다.
        self.u_hist = np.zeros((self.sgf_window // 2, 3))
        # 진단용 -- 매 step()이 덮어쓴다. 제어 노드가 주기마다 CSV로 기록한다.
        self.lam = 1.0                               # 이번 주기의 softmax 온도 (매번 풀린다)
        self.last_eta = float(cfg.K)
        self.last_n_hit, self.last_fallback, self.last_is_std = 0, False, 0.0
        self.last_hit = None
        # step() 이 한 번도 안 돌았으면 last_terms 는 None 이다. 제어 노드가 그것으로
        # '이 tick 에 solve 가 돌았는가'를 판단해 진단 칸을 비운다.
        self.last_terms = None
        self.last_lam_hit, self.last_best = False, 0
        self.last_S, self.last_w_bins = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0)
        # 샘플 블록 경계. ancillary 가 없으면 블록 하나다.
        self._groups = (cfg.K,)
        self.last_w_groups, self.last_is_groups = (1.0,), (0.0,)
        self.last_hit_groups = (0.0,)
        self.rng = np.random.default_rng(cfg.seed)   # np.random 전역 상태와 무관 -> 에피소드가 재현된다

    def reset(self):
        """Drop the warm start. Provided for reuse across episodes; runner.py
        builds a fresh MPPI per episode, so nothing calls this today."""
        # rng는 되감지 않는다 -- 재현성은 재구성으로만 보장된다 (Crowd.reset()과 다르다).
        self.U[:] = 0.0
        self.u_prev[:] = 0.0
        self.u_hist[:] = 0.0

    # 상태비용 가중치의 런타임 교체. cfg는 그대로 두고 __init__이 펴 둔 캐시만 바꾼다.
    # 주행과 제자리 회전이 같은 비용 항을 다른 무게로 쓰기 위한 것으로, 새 항은 없다.
    WEIGHTS = ('w_goal', 'w_obs', 'w_dyn', 'w_head', 'w_dir')

    def set_weights(self, **kw):
        """Swap state-cost weights in place, for a mode change.

        이름을 화이트리스트로 검사한다: 오타가 조용히 무시되면 가중치가 안 바뀐 채
        계속 도는데, 그 증상은 '회전이 느리다' 정도로만 보여서 찾기 어렵다.
        """
        for k, v in kw.items():
            assert k in self.WEIGHTS, f'{k!r} is not a weight; have {self.WEIGHTS}'
            setattr(self, k, float(v))

    def _solve_lam(self, S_state, I, keep):
        """Softmax temperature putting eta in [eta_min, eta_max], by bisection on log10(lam).

        Every exponent is (S_state + is_coeff*I)/lam for a fixed is_coeff, so all the
        gaps shrink together as lam grows and eta climbs monotonically from 1 to the
        number of surviving rollouts. That monotonicity is what makes bisection
        valid, and it is the whole reason is_coeff is not the temperature.

        Costs about 0.2 ms: 40 exponentials over K, against a 100 ms budget.

        돌려주는 둘째 값은 '밴드에 못 넣었다'다. 못 넣으면 lam 이 탐색 상한 1e4 로
        나오고, 그 온도의 softmax 는 거의 균등해져 갱신이 '랜덤 샘플의 평균'이 된다
        -- 그 tick 은 사실상 제어를 하지 않았다. 실기 로그에서 1 회 관측됐는데,
        성공한 solve 와 구분할 방법이 없었다. 호출부는 step() 한 곳뿐이다 (grep 확인).
        """
        target = np.sqrt(self.eta_min * self.eta_max)       # geometric centre of the band
        lo, hi = -3.0, 4.0                                  # lam in [1e-3, 1e4]
        lam = self.lam
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            lam = 10.0 ** mid
            S = S_state + self.is_coeff * I
            if keep is not None:
                S = S[keep]
            eta = np.exp(-(S - S.min()) / lam).sum()
            if self.eta_min <= eta <= self.eta_max:
                return lam, False
            lo, hi = (mid, hi) if eta < target else (lo, mid)
        return lam, True

    def _limit(self, A):
        """Actuator limit on (..., 3) velocities -> same shape.

        (vx, vy) is projected radially onto the disc of radius v_max[0] -- the
        Euclidean nearest point, so the direction of travel survives - and omega
        keeps its box. The disc is convex, so step (8)'s convex combination stays
        inside it and the guard after it stays a rounding guard, not a constraint.
        """
        out = np.empty_like(A)
        v = A[..., :2]
        n = np.linalg.norm(v, axis=-1, keepdims=True)
        out[..., :2] = v * np.minimum(1.0, self.v_max[0] / np.maximum(n, 1e-12))
        out[..., 2] = np.clip(A[..., 2], -self.v_max[2], self.v_max[2])
        return out

    def _rollout(self, x0, V):
        """V: (K, T, 3) -> X: (K, T, 3). Pure integrator, zero discretisation error."""
        # 파워드 캐스터 홀로노믹 베이스라 글로벌 프레임에서 xdot = u가 그대로 성립하고,
        # 한 제어주기 안에서 명령 속도가 일정하므로 이 적분은 근사가 아니라 해석해다.
        # 롤아웃 안에서 MuJoCo를 돌리지 않는다: 캐스터 지연과 가속 한계는 플래너 바깥,
        # BaseController의 Ruckig OTG(mujoco_env.py)가 담당한다.
        return x0 + np.cumsum(V * self.dt, axis=1)   # x0 (3,)가 (K, T, 3)에 브로드캐스트된다

    def _control_cost(self, V):
        """Input-dependent stage cost. V: (K, T, 3) -> (K, T).

            q_u = 1/2 u^T R u + 1/2 du^T R_d du + w_dir (1 - cos dpsi) ||v||

        Zero to the last bit when r_v = r_w = rd_v = rd_w = w_dir = 0, so leaving
        the knobs off reproduces the pre-control-cost planner exactly.
        """
        # t=0의 u_{t-1}은 직전 solve가 실행한 명령이다. 지평선 안에서만 차분하면 매 solve마다
        # 페널티가 리셋되어, 제어주기 경계에서 꺾이는 것을 잡지 못한다.
        prev = np.empty_like(V)
        prev[:, 0] = self.u_prev
        prev[:, 1:] = V[:, :-1]
        dV = V - prev

        # (a) 입력 크기, (b) 입력 변화율. R, R_d가 대각이라 (3,)와의 내적이 u^T R u와 같다.
        q = 0.5 * (V ** 2 @ self.R + dV ** 2 @ self.R_d)

        # (c) 방향 전환 -- vx, vy로만 잰다. omega는 자체 회전이라 진행 방향과 무관하다.
        v, vp = V[..., :2], prev[..., :2]
        nv = np.linalg.norm(v, axis=-1)
        denom = nv * np.linalg.norm(vp, axis=-1)
        # 어느 한쪽이 정지면 사잇각이 정의되지 않는다 -> cos=1로 두어 항을 0으로 만든다.
        # 그대로 나누면 정지 상태에서 출발할 때마다 없는 방향 전환에 벌점이 붙는다.
        cos = np.divide((v * vp).sum(-1), denom, out=np.ones_like(denom), where=denom > 1e-12)
        # ||v||를 곱하는 것이 핵심: 저속에서는 방향이 흔들려도 싸고, 전속력에서만 비싸진다.
        return q + self.w_dir * (1.0 - np.clip(cos, -1.0, 1.0)) * nv

    def _cost(self, X, V, wp, goal_theta, cost_volume, obs_ref=0.0):
        """Discounted stage cost of every rollout. X, V: (K, T, 3) -> (K,).

        wp is the A* lookahead point, not the episode goal: MPPI never sees the
        goal, which is why the goal term is charged at every step instead of once
        at the horizon end.

        obs_ref: 장애물 항의 기준선. q_obs = w_obs * max(0, c(p) - obs_ref) 로
        **기준선보다 나빠지는 쪽만** 벌점을 매긴다. 목표가 장애물 가까이 있을 때 그
        셀의 비용이 목표 유인을 이겨 MPPI 가 목표에 들어가지 않는 것을 막기 위한
        것이고, 기준선은 호출부가 거리에 따라 램프해서 넣는다 (executor._solve).
        0.0 이면 비용이 항상 >= 0 이므로 max(0, c - 0.0) == c 가 정확히 성립해
        **이전과 비트 동일**이다 (test_obs_ref_zero_is_bit_identical).
        """
        p = X[..., :2]                                      # (K, T, 2)

        # (1) 목표 유인 -- 웨이포인트까지의 유클리드 거리. lookahead가 0.75 m라 보통 0~15.
        q_goal = self.w_goal * np.linalg.norm(p - wp, axis=-1)

        # (2) 정적 장애물 -- grid.bake()가 미리 구워 둔 EDT 기반 soft cost, [0, 10].
        #     d <= robot_radius인 곳은 10으로 평평해서 안쪽에는 기울기가 없다. 즉 MPPI는
        #     팽창 영역 바깥에서만 필드를 타고 내려올 수 있다.
        q_obs = self.w_obs * np.maximum(0.0, self.grid.lookup(p) - obs_ref)

        # (3) 동적 장애물 -- 보행자 예측 코스트 볼륨, [0, 1]. None이면 항 자체를 건너뛰어
        #     정적 전용 결과를 그대로 재현한다. 보행자끼리는 합이 아니라 max로 합성되므로
        #     군중 밀도와 무관하게 스텝당 최대 60이다 (cost_volume.py).
        q_dyn = (self.w_dyn * self._dyn_lookup(p, cost_volume)
                 if cost_volume is not None else 0.0)

        # (4) 헤딩 -- goal_theta는 에피소드 내내 고정된 절대 yaw다. 오차를 [-pi, pi)로 감싼다.
        dth = (X[..., 2] - goal_theta + np.pi) % (2 * np.pi) - np.pi
        q_head = self.w_head * np.abs(dth)

        # (5) 제어 비용 -- 유일하게 상태 X가 아니라 입력 V에 걸리는 항이다. 다른 항과 같은
        #     할인을 받도록 여기서 합친다 ((6)의 IS 항과 혼동하지 말 것: 그건 무할인이다).
        q_ctrl = self._control_cost(V)

        # (6) 진단 -- 항별 할인합을 (K,) 로 남긴다. 합만 보면 '비싸다'까지만 알고 어느
        #     항이 최적 샘플을 골랐는지는 모른다. 4 회 reduction = ~50 us.
        #     q_dyn 은 **일부러 빼 둔다**: cost_volume=None 이 기본이고 보행자 항은 지금
        #     진단 대상이 아니다. 그래서 볼륨이 있으면 네 항의 합 < 반환값이다.
        #     broadcast_to: _control_cost 는 스칼라를 돌려줄 수 있다. 가중치를 모두 0 으로
        #     둔 구성을 재현하는 테스트가 그것을 0.0 으로 바꿔치기하고, 반환값 쪽은
        #     브로드캐스트로 받아 주지만 (K,) 로 접는 이 쪽은 축이 없어 터진다.
        _ctrl = np.broadcast_to(q_ctrl, q_goal.shape)       # 배열이면 그대로인 view
        self.last_terms = {'goal': (q_goal * self.disc).sum(axis=1),
                           'obs': (q_obs * self.disc).sum(axis=1),
                           'head': (q_head * self.disc).sum(axis=1),
                           'ctrl': (_ctrl * self.disc).sum(axis=1)}

        # disc는 (T,)라 (K, T)의 시간축에 정렬된다 -- 샘플별 가중치가 아니라 스텝별 할인이다.
        # 더하는 순서는 위 (1)~(5) 와 같고, 0.0 을 더하는 것은 IEEE754 에서 정확하므로
        # 이 반환값은 항을 누적하던 이전 구현과 비트 단위로 같다 (test_mppi.py 가 고정).
        return ((q_goal + q_obs + q_dyn + q_head + q_ctrl) * self.disc).sum(axis=1)

    def _dyn_lookup(self, p, cv):
        """cv: (n_steps, H, W) in [0, 1], same origin/res as the static grid.

        p must be (K, T, 2) with the horizon on axis 1: the volume's first axis is
        indexed by horizon step, and t below broadcasts as (1, T) against (K, T).
        """
        idx = np.floor((p - self.grid.origin) / self.grid.res).astype(np.int32)
        n, H, W = cv.shape
        c = np.clip(idx[..., 0], 0, W - 1)                  # 열 = x
        r = np.clip(idx[..., 1], 0, H - 1)                  # 행 = y (occ는 [row, col] = [y, x])
        # 볼륨이 T보다 짧으면 마지막 슬라이스를 반복한다 -- 예측이 얼어붙되 예외는 나지 않는다.
        # 레이턴시 보정은 여기가 아니라 볼륨 쪽에 있다: 나이 a인 스냅샷으로 만든 볼륨이 스스로
        # 시간축을 시프트하므로, 인덱스 t는 "지금부터 t 제어주기 후"를 뜻한다. 다만 X[..., t]는
        # (t+1)*dt 뒤의 상태라, 각 롤아웃 점은 한 스텝 이전의 예측으로 값이 매겨진다.
        t = np.clip(np.arange(self.T), 0, n - 1)[None, :]
        return cv[t, r, c]                                  # (K, T)

    def step(self, x0, wp, goal_theta=0.0, cost_volume=None, ancillary=None, obs_ref=0.0):
        """One solve. x0: (3,), wp: (2,), cost_volume: (n_steps, H, W) or None
        -> u: (3,) global-frame velocity command.

        ancillary: (A, T, 3) or None. Biased-MPPI 의 보조 제어열이다 (경로 추종, 측면
        기동 등). 들어오면 가우시안 블록을 줄이고 그 자리에 각 열의 노이즈 사본을 넣는다.
        **A = 0 또는 None 이면 아래 (1)(2) 가 예전 경로를 그대로 타고, 난수 호출의 모양과
        순서까지 같아서 결과가 비트 단위로 같다** (test_mppi.py 가 골든으로 고정한다).
        """
        A = 0 if ancillary is None else len(ancillary)
        if A == 0:
            # (1) 이전 해 U를 중심으로 K x T개의 섭동을 한 번에 뽑는다. Sigma가 대각이라
            #     (3,) sigma를 scale로 넘기는 것으로 충분하다 -- 공분산 행렬을 만들지 않는다.
            E = self.rng.normal(0.0, self.sigma, size=(self.K, self.T, 3))
            # (2) 샘플 입력열. 액추에이터 한계로 클리핑해서, 실기가 못 하는 움직임은
            #     애초에 후보에 올리지 않는다.
            V = self._limit(self.U + E)
            self._groups = (self.K,)
        else:
            c = self.anc_copies
            kg = self.K - A * c
            # K 를 줄이면서 anc_copies 를 그대로 두면 가우시안 블록이 급격히 작아진다.
            # eta 밴드가 K 를 따라가지 않는 것과 같은 종류의 함정이라 여기서 막는다.
            assert kg >= self.K // 2, (
                f'ancillary {A} x {c} = {A * c} of K={self.K} leaves {kg} gaussian '
                f'samples; keep at least half')
            anc = np.asarray(ancillary, float)
            assert anc.shape == (A, self.T, 3), \
                f'ancillary must be (A, {self.T}, 3), got {anc.shape}'
            E = self.rng.normal(0.0, self.sigma, size=(kg, self.T, 3))
            # 사본 0 은 노이즈 0 -- 의도한 기동 그 자체가 반드시 후보에 있어야 한다.
            Ea = np.zeros((A, c, self.T, 3))
            Ea[:, 1:] = self.rng.normal(0.0, self.anc_sigma, size=(A, c - 1, self.T, 3))
            # ancillary 도 같은 _limit 을 거친다. 디스크가 볼록이므로 (8) 의 볼록결합이
            # 한계 안에 남고, 그 뒤의 _limit 은 여전히 반올림 가드다.
            V = self._limit(np.concatenate(
                [self.U + E, (anc[:, None] + Ea).reshape(A * c, self.T, 3)]))
            self._groups = (kg,) + (c,) * A

        # (3) 클리핑 후 노이즈를 다시 정의한다. 논문과 다른 유일한 지점이고, 두 군데서 필요하다:
        #     (a) (6)의 IS 항은 '실제로 적용된' 섭동에 대해서만 옳다. 포화된 샘플에서는
        #         원래 뽑은 E가 그대로 적용되지 않았다.
        #     (b) (8)의 갱신이 U <- sum_k w^k V^k, 즉 이미 클리핑된 열들의 볼록결합이 된다.
        #         원래 E를 쓰면 갱신이 포화 방향으로 편향되어 U가 ±v_max에 붙어 버린다 (넘친 몫은
        #         아래 클립이 잘라낸다) -- 롤아웃이 실행 불가능한 명령의 대가를 보여주지 않으므로
        #         되돌리는 압력이 없다.
        #     대가: E가 절단정규분포가 되어 IS 가중치는 근사가 된다.
        #
        #     ancillary 샘플에도 **같은 정의를 쓴다**. 그 샘플은 U + eps 꼴이 아니므로
        #     eps 가 커지고, (6) 의 IS 항이 그만큼 큰 고정 벌점을 매긴다 -- U 와 같은
        #     방향인 ancillary 는 I > 0 으로 벌점, 반대 방향은 I < 0 으로 보상이다.
        #     직관과 반대지만 모든 샘플에 일관되게 적용된다. 블록별 평균을
        #     last_is_groups 로 내보내 그 편향을 사후에 직접 보게 해 두었다.
        E = V - self.U

        # (4) 샘플별 궤적을 벡터화 한 번으로 굴린다.
        X = self._rollout(x0, V)

        # (5) 할인된 단계별 비용 (상태 항 + 제어 항).
        S_state = self._cost(X, V, wp, goal_theta, cost_volume, obs_ref)

        # (6) importance sampling 항: lambda * sum_t u_t^T Sigma^-1 eps^k_t.
        #     대각 Sigma에서는 이 einsum이 그 식과 문자 그대로 같다 (t, d를 접고 k만 남긴다).
        #     부호는 +, 계수는 lambda다 -- lambda/2가 아니다. 여기에는 할인을 걸지 않는다.
        #     계수는 is_coeff이고 (7)의 온도와 분리되어 있다. 같았다면 지수에서 둘이 상쇄되어
        #     IS 항이 온도와 무관해지고, 순항에서 그 샘플 간 편차(std ~ 4.8)가 eta를 ~2에 묶는다.
        #     분리한 대가로 이 항은 엄밀한 중요도 보정이 아니라 고정 가중 제어 페널티가 된다.
        I = np.einsum('td,ktd,d->k', self.U, E, 1.0 / self.sigma ** 2)
        self.last_is_std = float(I.std())
        # 진단: inf 마스킹 **전**의 샘플 비용 분포. 넓으면 _solve_lam 이 천장을 친다.
        _S = S_state + self.is_coeff * I
        self.last_S = (float(_S.min()), float(np.median(_S)), float(_S.max()))

        # (6a) 충돌 롤아웃 제외. _cost()가 이미 읽은 정적 비용을 한 번 더 읽는다 -- K*T 조회라
        #      싸고, _cost()의 반환형을 바꾸지 않는 쪽을 택했다. 가중치 0은 비용 +inf와 같으므로
        #      갱신의 가중합 형태와 beta = min S는 그대로이고, min은 살아남은 롤아웃 위에서 잡힌다.
        keep = None
        #      obs_ref 와 무관하게 **원래 비용**으로 본다. 기준선을 깎는 것은 '얼마나
        #      나쁜가'의 눈금이고, '들어가면 안 되는 곳'은 눈금과 상관없이 그대로다.
        hit = (self.grid.lookup(X[..., :2]) >= LETHAL).any(axis=1)
        self.last_hit, self.last_n_hit = hit, int(hit.sum())
        self.last_fallback = bool(hit.all())         # 전부 적중이면 전체를 가중한다 (0으로 나누지 않으려고)
        if hit.any() and not hit.all():
            keep = ~hit

        # (6b) 이번 주기의 비용 분포로 온도를 푼다 -- eta를 [eta_min, eta_max]에 넣는 값.
        self.lam, self.last_lam_hit = self._solve_lam(S_state, I, keep)
        S = S_state + self.is_coeff * I
        if keep is not None:
            S = np.where(keep, S, np.inf)

        # (7) beta = min_k S^k를 뺀 softmax. 상수 시프트라 값은 그대로지만 선택이 아니다:
        #     S는 스텝당 상한이 260 + 목표항이라 보통 수백~수천이고, 온도가 낮으면 exp가
        #     0으로 underflow한다 -> w.sum()이 0 -> U가 NaN. min을 빼면 최선이 항상 1이다.
        #     +inf가 들어간 적중 롤아웃은 여기서 정확히 0이 되고, min은 생존자 위에서 잡힌다.
        w = np.exp(-(S - S.min()) / self.lam)               # (K,)
        # eta = sum_k exp(...) in [1, K]는 '유의미한 샘플이 몇 개인가'다. 1에 붙으면 갱신이
        # 사실상 노이즈 샘플 하나 고르기가 되어 명령이 스텝마다 튄다 -- 실제로 이 튜닝의 측정값이
        # eta ~ 1.0이었다. 그래서 mppi_torch처럼 lam을 되먹임해서 eta를 [eta_min, eta_max]에 둔다.
        # eta = sum_k exp(...) in [1, K]는 '유의미한 샘플이 몇 개인가'다. (6b)가 이 값을
        # 목표 구간에 넣어 두었으므로 여기서는 기록만 한다.
        eta = w.sum()
        w /= eta
        self.last_eta = float(eta)
        self.last_best = int(np.argmin(S))      # 최적 '생존' 샘플 (탈락분은 +inf). last_terms 색인용
        # 가중치가 양립 불가능한 선택지로 쪼개졌는가. 롤아웃 순변위가 캐럿 방향과 이루는
        # 각으로 4 분할한 Sum(w): (fwd, left, back, right), 합 1. 이 MPPI 는 단일 가우시안
        # 에서 K 개를 뽑으므로 nominal/side+- 같은 샘플 그룹이 존재하지 않는다 -- 이것이
        # 그 역할을 대신한다. w_fwd 와 w_left 가 비슷하면 경로와 싸우는 중이다.
        _d = X[:, -1, :2] - x0[:2]
        _a = np.arctan2(_d[:, 1], _d[:, 0]) - np.arctan2(wp[1] - x0[1], wp[0] - x0[0])
        _b = (((_a + np.pi / 4) % (2 * np.pi)) // (np.pi / 2)).astype(np.int8)
        self.last_w_bins = tuple(float(w[_b == i].sum()) for i in range(4))
        # 블록별 Sum(w) 와 IS 항 평균. 두 질문은 다르다 -- 전자는 '어느 후보가 이겼는가'
        # (가중치), 후자는 'IS 항이 그 블록을 체계적으로 밀어내는가'(비용)다. 방향
        # 분할(last_w_bins)과도 직교한다: 저쪽은 '어디로 가려는가'다.
        _e = np.cumsum((0,) + self._groups)
        self.last_w_groups = tuple(float(w[a:b].sum()) for a, b in zip(_e[:-1], _e[1:]))
        self.last_is_groups = tuple(float((self.is_coeff * I[a:b]).mean())
                                    for a, b in zip(_e[:-1], _e[1:]))
        # 블록별 lethal 마스킹 비율. Sum(w) 가 0 인 것이 '비용이 높아 진 것'인지
        # '전부 탈락한 것'인지 이것 없이는 구분할 수 없다 -- 실제로 ancillary 의
        # Sum(w) = 0.000 을 비용 패배로 잘못 읽었고, n_hit 이 2 -> 65 로 뛴 것(= 블록
        # 크기 63)을 보고서야 전원 탈락임을 알았다.
        self.last_hit_groups = tuple(float(hit[a:b].mean()) for a, b in zip(_e[:-1], _e[1:]))

        # (8) 궤적이 아니라 '노이즈'의 가중평균을 이전 명목해에 더한다. sum_k w^k = 1이고
        #     (3)에서 E = V - U로 맞춰 두었으므로 결과는 sum_k w^k V^k와 같다.
        #
        #     이 등가는 ancillary 처럼 **U + eps 꼴이 아닌 샘플에도 성립한다**:
        #         U + sum_k w^k (V^k - U) = sum_k w^k V^k - U sum_k w^k + U = sum_k w^k V^k
        #     필요한 것은 sum_k w^k = 1 과 (3) 의 재정의뿐이고 둘 다 무조건 성립한다
        #     (탈락 롤아웃은 w = 0 이다). 그래서 Biased-MPPI 를 넣으면서 이 줄을
        #     sum_k w^k V^k 로 고칠 필요가 없었다. 실측: 가중치를 ancillary 블록에만
        #     몰면 갱신이 정확히 그 블록의 평균이 되고, 두 식의 차이는 1.3e-15
        #     (U 스케일 대비 2.9e-15) 로 반올림뿐이다. **고쳐 쓰면 그 1e-15 때문에
        #     'ancillary 0 개면 비트 동일' 불변 조건이 깨진다.**
        self.U += np.einsum('k,ktd->td', w, E)
        #     볼록결합이니 수학적으로는 이미 한계 안이지만, K=512의 float64 누적으로 몇 ULP
        #     벗어날 수 있다. 죽은 코드가 아니라 반올림 가드다.
        self.U = self._limit(self.U)

        # 시각화 전용 (viz.py). last_X는 이미 있는 배열을 가리키기만 한다.
        # last_nominal만 롤아웃을 한 번 더 도는데, (9)의 시프트 '전에' 계산해야 인덱스 0이
        # 방금 반환할 u와 맞는다. viz는 last_X의 존재로 '한 번은 풀었는지'를 판단한다.
        # last_w는 노출만 하고 아무도 읽지 않는다 -- 위의 K_eff 때문에 viz가 쓰지 않는다.
        self.last_X, self.last_w = X, w
        self.last_nominal = self._rollout(x0, self.U[None])[0]

        # (9) 명령. [실행된 과거 h개; 계획 U[0..h]]의 가운데 값이고, U는 읽기만 한다 --
        #     필터 결과는 warm start로 돌아가지 않는다. 과거 절반이 들어가므로 창이 solve
        #     경계를 가로지르고, 그래서 연속된 명령끼리 이어진다 (이게 jerk의 3.4배를 줬다).
        #     다항 적합이라 한계를 넘을 수 있어 _limit을 다시 건다.
        #     nav2_mppi도 같은 창에 같은 4개 이력을 쓰지만 결과를 control_sequence_에 되쓴다.
        if self.sgf_window:
            seq = np.vstack([self.u_hist, self.U[:len(self.u_hist) + 1]])
            u = self._limit((self._sgf_c @ seq)[None])[0]
            self.u_hist = np.vstack([self.u_hist[1:], u])
        else:
            u = self.U[0].copy()
        self.u_prev = u.copy()      # 실제로 실행되는 명령. 다음 solve의 rate 비용에서 u_{t-1}이 된다

        # (10) Receding horizon: 열을 한 칸 당기고 말단은 직전 값으로 채운다. 이 시프트가
        #      warm start이고, 그래서 K=512로 충분하다 -- 매 solve가 이전 계획을 이어 다듬는다.
        #      말단을 0으로 채우면 계획이 매번 '끝에서 급정거'로 끝나고, 지평선이 미끄러지며
        #      그 불연속이 앞으로 밀려온다. np.roll이 새 배열을 만드므로 위의 .copy()는 반환값이
        #      (T, 3) 버퍼를 붙잡지 않게 하는 것이다.
        self.U = np.roll(self.U, -1, axis=0)
        self.U[-1] = self.U[-2]
        return u
1