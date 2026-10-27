import time

import numpy as np

from nav.grid import Grid, static_grid
from nav.mppi import MPPI, MPPIConfig


def _empty():
    return Grid(origin=(-1.0, -1.0), res=0.05, shape=(120, 120)).bake(robot_radius=0.35)


def _roll(mppi, x, goal, n=300, cost_volume=None):
    for _ in range(n):
        u = mppi.step(x, goal[:2], cost_volume=cost_volume)
        x = x + u * mppi.dt
        if np.linalg.norm(x[:2] - goal[:2]) < 0.15:
            break
    return x


def test_converges_in_free_space():
    mppi = MPPI(_empty())
    x = _roll(mppi, np.zeros(3), np.array([3.0, 2.0]))
    assert np.linalg.norm(x[:2] - np.array([3.0, 2.0])) < 0.15


def test_respects_velocity_limits():
    mppi = MPPI(_empty())
    for _ in range(20):
        u = mppi.step(np.zeros(3), np.array([50.0, 50.0]))   # saturating waypoint
        assert np.all(np.abs(u) <= mppi.v_max + 1e-9)


def test_beta_subtraction_survives_huge_costs():
    """S is O(1e6) here; without subtracting min(S) the weights underflow to 0
    and the update becomes NaN."""
    mppi = MPPI(_empty(), MPPIConfig(w_goal=1e5))
    u = mppi.step(np.zeros(3), np.array([500.0, 500.0]))
    assert np.all(np.isfinite(u))
    assert np.linalg.norm(u[:2]) > 0.1                      # still drives towards the goal


def test_seed_is_reproducible():
    a = _roll(MPPI(_empty(), MPPIConfig(seed=3)), np.zeros(3), np.array([2.0, 1.0]))
    b = _roll(MPPI(_empty(), MPPIConfig(seed=3)), np.zeros(3), np.array([2.0, 1.0]))
    assert np.allclose(a, b)


def test_avoids_a_wall():
    grid = static_grid()
    mppi = MPPI(grid)
    x = _roll(mppi, np.array([2.0, 1.0, 0.0]), np.array([2.5, 1.0]), n=60)
    assert grid.lookup(x[:2]) < 10.0                        # never entered the wall


def test_step_time_budget():
    mppi = MPPI(static_grid())
    x, wp = np.array([1.0, 1.0, 0.0]), np.array([4.0, 4.0])
    mppi.step(x, wp)                                        # warm up
    t0 = time.perf_counter()
    for _ in range(20):
        mppi.step(x, wp)
    ms = 1e3 * (time.perf_counter() - t0) / 20
    assert ms < 20.0, f'MPPI step took {ms:.1f} ms (K=512, T=25)'


def _blob(grid, xy, half_cells=10, n_steps=25):
    cv = np.zeros((n_steps,) + grid.occ.shape, np.float32)
    c, r = grid.to_cell(*xy)
    cv[:, r - half_cells:r + half_cells, c - half_cells:c + half_cells] = 1.0
    return cv


def test_dyn_lookup_reads_the_right_cells():
    """p must be (K, T, 2): the volume's first axis is indexed by horizon step."""
    grid = static_grid()
    mppi = MPPI(grid)
    cv = _blob(grid, (2.0, 1.0))                            # +-0.5 m around (2, 1)
    pts = np.zeros((2, mppi.T, 2))
    pts[0] = [2.0, 1.0]                                     # inside the blob
    pts[1] = [3.5, 1.0]                                     # outside
    got = mppi._dyn_lookup(pts, cv)
    assert got.shape == (2, mppi.T)
    assert np.all(got[0] == 1.0) and np.all(got[1] == 0.0)


def test_cost_volume_steers_away_from_the_blob():
    """Same seed, same goal: the only difference is the dynamic cost. The robot
    should spend less time inside the blob when it can see it."""
    # Inside CROWD_AREA: no static wall within reach, so the only thing that can
    # bend the path is the blob. Run it against a wall and the wall's inflation
    # moves the robot further than the blob does.
    grid, goal = static_grid(), np.array([7.5, 1.0])
    cv = _blob(grid, (5.6, 1.0), half_cells=8)              # ~0.8 m pedestrian-sized
    exposure = {}
    for name, arg in [('plain', None), ('aware', cv)]:
        mppi, x = MPPI(grid, MPPIConfig(seed=1)), np.array([4.3, 1.0, 0.0])
        hits = 0
        for _ in range(70):
            x = x + mppi.step(x, goal, cost_volume=arg) * mppi.dt
            c, r = grid.to_cell(x[0], x[1])
            hits += cv[0, r, c] > 0
        exposure[name] = hits
        assert np.all(np.isfinite(x))
    assert exposure['plain'] > 0, 'the blind run must cross the blob or this proves nothing'
    assert exposure['aware'] < exposure['plain'], exposure


def _ctrl(T=3, **kw):
    return MPPI(_empty(), MPPIConfig(T=T, **kw))


def test_control_cost_vanishes_at_zero_weights():
    """With all five knobs at 0 the term must be zero to the last bit, so turning
    it off reproduces the pre-control-cost planner exactly."""
    off = MPPIConfig(r_v=0.0, r_w=0.0, rd_v=0.0, rd_w=0.0, w_dir=0.0)
    a, b = MPPI(_empty(), off), MPPI(_empty(), off)
    a._control_cost = lambda V: 0.0
    x, goal = np.array([1.0, 1.0, 0.0]), np.array([3.0, 2.0])
    for _ in range(30):
        ua, ub = a.step(x, goal), b.step(x, goal)
        assert np.array_equal(ua, ub)
        x = x + ua * a.dt


def test_control_cost_matches_the_formula():
    m = _ctrl(r_v=2.0, r_w=3.0, rd_v=5.0, rd_w=7.0, w_dir=11.0)
    m.u_prev = np.array([0.3, 0.0, 0.1])
    V = np.array([[[0.3, 0.0, 0.1],                 # same as u_prev: du = 0, dpsi = 0
                   [0.0, 0.4, 0.0],                 # 90 deg turn
                   [-0.2, 0.0, 0.0]]])              # another 90 deg turn
    R, Rd = np.array([2.0, 2.0, 3.0]), np.array([5.0, 5.0, 7.0])
    prev = [m.u_prev, V[0, 0], V[0, 1]]
    want = []
    for t in range(3):
        u, up = V[0, t], prev[t]
        nv, nvp = np.linalg.norm(u[:2]), np.linalg.norm(up[:2])
        cos = 1.0 if nv * nvp < 1e-12 else u[:2] @ up[:2] / (nv * nvp)
        want.append(0.5 * (R * u ** 2).sum() + 0.5 * (Rd * (u - up) ** 2).sum()
                    + 11.0 * (1 - cos) * nv)
    assert np.allclose(m._control_cost(V)[0], want, rtol=0, atol=1e-12)


def test_direction_term_is_zero_from_rest():
    """cos dpsi is undefined against a zero velocity; without the guard the robot
    would be charged for a direction change every time it starts moving."""
    m = _ctrl(T=1, w_dir=100.0)
    assert m._control_cost(np.array([[[0.5, 0.0, 0.0]]]))[0, 0] == 0.0


def test_rate_penalty_prefers_the_smooth_sequence():
    m = _ctrl(T=4, rd_v=10.0)
    m.u_prev = np.array([0.5, 0.0, 0.0])
    smooth = np.full((1, 4, 3), 0.0)
    smooth[0, :, 0] = 0.5                                    # hold the command
    zigzag = np.zeros((1, 4, 3))
    zigzag[0, :, 0] = [0.5, -0.5, 0.5, -0.5]                 # same speed, flipping
    assert m._control_cost(smooth).sum() < m._control_cost(zigzag).sum()


def test_direction_penalty_ignores_omega():
    """c_dir is built from vx, vy only: spinning in place is not a direction change."""
    m = _ctrl(T=2, w_dir=50.0)
    m.u_prev = np.array([0.4, 0.0, 0.0])
    straight = np.zeros((1, 2, 3))
    straight[0, :, 0] = 0.4
    spinning = straight.copy()
    spinning[0, :, 2] = 1.0                                  # same track, now yawing
    assert np.allclose(m._control_cost(straight), m._control_cost(spinning))


def test_u_prev_tracks_the_executed_command():
    """t=0 of the next solve differences against what was actually executed, so the
    penalty spans control periods instead of resetting at every solve."""
    m = MPPI(_empty(), MPPIConfig(rd_v=1.0))
    assert np.all(m.u_prev == 0.0)
    u = m.step(np.zeros(3), np.array([2.0, 1.0]))
    assert np.array_equal(m.u_prev, u)
    m.reset()
    assert np.all(m.u_prev == 0.0)






def test_savgol_window_is_made_legal():
    """savgol_filter needs an odd window, wider than the order and no longer than
    the horizon; a bad config should disable the filter, not raise."""
    assert MPPI(_empty(), MPPIConfig(T=6, sgf_window=9)).sgf_window == 5
    assert MPPI(_empty(), MPPIConfig(sgf_window=8)).sgf_window == 9
    assert MPPI(_empty(), MPPIConfig(sgf_window=3, sgf_order=5)).sgf_window == 0
    MPPI(_empty(), MPPIConfig(T=4, sgf_window=9)).step(np.zeros(3), np.array([1.0, 1.0]))



def test_disc_limit_projects_translation_radially():
    """vxy_disc: (vx, vy) lands on the disc along its own direction, omega on its box,
    and a point already inside is left untouched."""
    m = MPPI(static_grid(), MPPIConfig(v_max=(0.5, 0.5, 1.0)))
    A = np.array([[0.8, 0.6, 2.0],      # |v| = 1.0 -> scaled by 0.5, omega clipped
                  [0.3, 0.4, -0.5],     # |v| = 0.5 exactly, inside -> unchanged
                  [0.1, -0.2, -3.0]])   # inside, only omega clipped
    out = m._limit(A)
    np.testing.assert_allclose(out[0], [0.4, 0.3, 1.0])
    np.testing.assert_allclose(out[1], A[1])
    np.testing.assert_allclose(out[2], [0.1, -0.2, -1.0])
    # A box-clipped diagonal would reach 0.707; the disc never exceeds its radius.
    x = np.array([0.0, 0.0, 0.0])
    for _ in range(30):
        u = m.step(x, np.array([5.0, 5.0]))
        assert np.hypot(u[0], u[1]) <= 0.5 + 1e-9
        x = x + u * m.dt




def test_rollouts_that_hit_get_zero_weight():
    """Beside the doorway wall some rollouts cross into it and some do not."""
    m = MPPI(static_grid(), MPPIConfig(K=256))
    m.step(np.array([2.55, 2.9, 0.0]), np.array([3.9, 4.0]))
    assert 0 < m.last_n_hit < m.K and not m.last_fallback
    assert np.all(m.last_w[m.last_hit] == 0.0)
    assert np.isclose(m.last_w.sum(), 1.0)


def test_all_rollouts_hitting_falls_back_to_all_of_them():
    """Inside a wall nothing is feasible; weigh everything rather than divide by zero."""
    m = MPPI(static_grid(), MPPIConfig(K=128))
    u = m.step(np.array([3.15, 1.0, 0.0]), np.array([5.0, 1.0]))
    assert m.last_fallback and m.last_n_hit == m.K
    assert np.all(np.isfinite(u)) and np.isclose(m.last_w.sum(), 1.0)


def test_solved_temperature_puts_eta_in_the_band_at_cruise():
    """The point of decouple_is: coupled, eta is stuck near 2 at speed whatever lam is."""
    cfg = MPPIConfig(K=512)
    m = MPPI(static_grid(), cfg)
    x, wp = np.array([3.9, 0.1, 0.0]), np.array([9.0, 4.3])
    etas = []
    for _ in range(15):
        u = m.step(x, wp)
        x = x + u * m.dt
        etas.append(m.last_eta)
    assert all(25.6 <= e <= 51.2 for e in etas), etas




def test_sgf_output_filters_the_command_and_leaves_the_warm_start_alone():
    """Same seed, one step: with sgf_output the warm start must be exactly what it is
    with no filter at all, and the command must be the centre of the 9-point window
    over [four executed commands (zero, from rest); the unfiltered plan U[0..4]]."""
    from scipy.signal import savgol_coeffs
    x, wp = np.array([3.9, 0.1, 0.0]), np.array([9.0, 4.3])
    a = MPPI(static_grid(), MPPIConfig(K=256))
    b = MPPI(static_grid(), MPPIConfig(K=256, sgf_window=0))
    ua, ub = a.step(x, wp), b.step(x, wp)
    assert np.array_equal(a.U, b.U)                     # warm start untouched by the filter
    plan = np.vstack([ub, b.U[:4]])                     # U[0..4] before the shift
    want = savgol_coeffs(9, 2) @ np.vstack([np.zeros((4, 3)), plan])
    np.testing.assert_allclose(ua, a._limit(want[None])[0], atol=1e-12)
    np.testing.assert_allclose(a.u_prev, ua)            # the rate cost sees what was executed
    np.testing.assert_allclose(a.u_hist[-1], ua)


def test_set_weights_swaps_the_cached_weight_and_rejects_typos():
    """회전 모드가 쓰는 유일한 훅. cfg는 그대로 둔 채 캐시만 바뀌어야 한다."""
    mppi = MPPI(_empty(), MPPIConfig(K=64))
    mppi.set_weights(w_head=40.0, w_dir=0.0)
    assert (mppi.w_head, mppi.w_dir) == (40.0, 0.0)
    assert mppi.cfg.w_head == MPPIConfig().w_head    # cfg는 불변
    try:
        mppi.set_weights(w_heading=1.0)              # 오타는 조용히 넘어가면 안 된다
    except AssertionError:
        return
    raise AssertionError('a misspelled weight was accepted')


def test_rotation_weights_turn_in_place():
    """제자리 회전: wp를 현재 위치로 두고 w_head를 올리면 yaw만 움직인다."""
    mppi = MPPI(_empty(), MPPIConfig(K=256))
    mppi.set_weights(w_head=40.0, w_goal=60.0, w_dir=0.0)
    x = np.zeros(3)
    for _ in range(60):
        u = mppi.step(x, x[:2].copy(), goal_theta=np.pi / 2)
        x = x + u * mppi.dt
    assert abs(x[2] - np.pi / 2) < 0.25, f'yaw {x[2]:.3f}'
    assert np.linalg.norm(x[:2]) < 0.15, f'drifted to {x[:2]}'


# ── _cost 항 분리 리팩터의 회귀 고정 ─────────────────────────────────────────
# 아래 두 배열은 **리팩터 전** 구현(_cost 가 q 에 누적하던 버전)으로 2026-10-06 에
# 뽑은 값이다. 리팩터가 반환값을 바꾸지 않았다는 것은 구현끼리 비교해서는 알 수
# 없으므로 (한쪽이 사라졌다) 고정값으로 박아 둔다. 비트 동일을 요구한다.
_G_KW = dict(K=16, T=5, seed=7, eta_min=0.8, eta_max=1.6,
             r_v=0.3, r_w=0.2, rd_v=0.4, rd_w=0.1, w_dir=0.25)
_G_WP = np.array([0.42, -0.31])
_G_THETA = 0.37

S_NONE = np.array([
    599.025635522338, 557.2629560792607, 422.4553238689183, 526.8194372824345,
    414.12441038530176, 405.754323289147, 408.03334437482647, 400.2544905567658,
    486.24794348261173, 695.8181525658528, 659.8101256362439, 775.2408928759693,
    793.3886489965892, 399.65716127805626, 283.4030039894651, 566.5398912334681,
])
S_DYN = np.array([
    748.795552322338, 761.8302957498489, 555.4514661748005, 684.6978269294933,
    561.5009297264781, 573.7172294538528, 566.1941428689441, 463.72918740382465,
    586.0185976002588, 765.3899805187939, 792.6400817774204, 886.0488155112633,
    927.6923395377656, 546.0097570427621, 392.8930956600533, 685.0350127393505,
])


class _GoldenGrid:
    """결정적 합성 비용장. bake() 를 쓰지 않는다 -- 골든 값이 grid.py 의 변경에
    묶이면 이 테스트가 _cost 가 아니라 grid 를 재는 테스트가 된다."""
    origin, res = np.array([-2.0, -2.0]), 0.05

    def __init__(self):
        self.c = np.fromfunction(lambda r, c: ((r * 7 + c * 13) % 23) / 2.3, (80, 80))

    def lookup(self, pts):
        i = np.floor((pts - self.origin) / self.res).astype(np.int32)
        return self.c[np.clip(i[..., 1], 0, 79), np.clip(i[..., 0], 0, 79)]


def _golden():
    """(mppi, X, V) -- 골든 값을 뽑을 때와 똑같은 입력."""
    m = MPPI(_GoldenGrid(), MPPIConfig(**_G_KW))
    rng = np.random.default_rng(3)
    X = rng.normal(0, 0.4, (m.K, m.T, 3))
    V = rng.normal(0, 0.2, (m.K, m.T, 3))
    m.u_prev = np.array([0.11, -0.07, 0.03])
    return m, X, V


def _golden_volume(m):
    return np.fromfunction(lambda t, r, c: ((t * 3 + r * 5 + c * 11) % 17) / 17.0,
                           (m.T, 80, 80))


def test_cost_bit_identical_after_term_split():
    """항을 따로 들고 더하도록 바꾼 리팩터가 반환값을 바꾸지 않았다."""
    m, X, V = _golden()
    np.testing.assert_array_equal(m._cost(X, V, _G_WP, _G_THETA, None), S_NONE)
    np.testing.assert_array_equal(
        m._cost(X, V, _G_WP, _G_THETA, _golden_volume(m)), S_DYN)


def test_last_terms_sum():
    """last_terms 네 항의 합 == 반환값. **q_dyn 은 last_terms 에 없다**: 볼륨이
    없을 때만 일치하고, 있으면 차이가 정확히 q_dyn 의 할인합이다."""
    m, X, V = _golden()
    S = m._cost(X, V, _G_WP, _G_THETA, None)
    assert np.allclose(sum(m.last_terms.values()), S)

    S2 = m._cost(X, V, _G_WP, _G_THETA, _golden_volume(m))
    tot = sum(m.last_terms.values())
    # q_dyn >= 0 이므로 모든 샘플에서 작거나 같고, 적어도 하나는 엄격히 작다.
    assert np.all(tot <= S2)
    assert np.any(tot < S2)


def test_solve_lam_reports_failure():
    """밴드에 넣으면 (lam, False), 못 넣으면 (탐색 상한 근방, True)."""
    m, X, V = _golden()
    S_state = m._cost(X, V, _G_WP, _G_THETA, None)
    I = np.zeros(m.K)

    lam, hit = m._solve_lam(S_state, I, None)
    assert not hit, 'eta 를 밴드에 넣었는데 실패로 보고했다'
    assert m.eta_min <= np.exp(-(S_state - S_state.min()) / lam).sum() <= m.eta_max

    # K=16 에서 eta 가 절대 닿을 수 없는 밴드를 요구한다 -> 상한에서 끝난다.
    m.eta_min, m.eta_max = 15.9, 16.0
    lam, hit = m._solve_lam(S_state, I, None)
    assert hit, '밴드에 못 넣었는데 성공으로 보고했다'
    assert lam > 1e3, f'상한 근방이어야 한다, got {lam}'


def test_step_fills_diagnostics():
    """step() 한 번으로 진단 속성이 모두 채워진다. 노드가 이것을 CSV 로 찍는다."""
    # K 는 기본값으로 둔다. eta_min/eta_max (25.6/51.2) 는 K=512 의 5~10 % 로 고른
    # 값이라, K 를 줄이면 밴드가 K 의 수십 % 가 되어 softmax 가 거의 균등해지고 명령이
    # 안 나온다 -- K=64 로 짰다가 20 step 에 0.15 m 만 전진하는 것을 보고 알았다.
    m = MPPI(_empty(), MPPIConfig())
    assert m.last_terms is None, 'solve 전에는 None 이어야 한다 (노드의 빈칸 판정 근거)'
    # 첫 solve 는 U = 0 에서 출발하므로 순변위가 순수 노이즈다 -- 방향 분할이 고르게
    # 퍼지는 것이 정상이다. 분할이 쏠리는지는 warm start 가 붙은 뒤에 본다.
    x = np.zeros(3)
    for _ in range(20):
        x = x + m.step(x, np.array([3.0, 0.0])) * m.dt
    assert set(m.last_terms) == {'goal', 'obs', 'head', 'ctrl'}
    assert all(v.shape == (m.K,) for v in m.last_terms.values())
    assert 0 <= m.last_best < m.K
    assert m.last_S[0] <= m.last_S[1] <= m.last_S[2]
    assert abs(sum(m.last_w_bins) - 1.0) < 1e-9, m.last_w_bins
    # 자유 공간을 +x 로 달리는 중이면 가중치는 fwd 로 쏠린다.
    assert m.last_w_bins[0] > 0.5, m.last_w_bins
    assert x[0] > 0.5, f'달리지 않았다: {x}'


# ── Biased-MPPI (arXiv:2401.09241) ───────────────────────────────────────────
# step() 의 비트 동일성 골든. ancillary 를 넣기 **전** 코드로 2026-10-06 에 뽑았다.
# ancillary=None 경로는 난수 호출의 모양과 순서까지 예전과 같아야 한다.
STEP_U = np.array([
    [0.0415278733744521, 0.028265809909708565, 0.044360578957793446],
    [0.08069986775196362, 0.036768559278843184, 0.07746001122323502],
    [0.11276985709765566, 0.056970814170020485, 0.09023181904219837],
    [0.1595626354845909, 0.07764973276673416, 0.07176541974341698],
    [0.19482917941660735, 0.0876983103555497, 0.05320824946344469],
])


def _biased():
    return (MPPI(_empty(), MPPIConfig(seed=11)),
            np.array([0.5, 0.5, 0.0]), np.array([2.5, 1.5]))


def test_ancillary_none_is_bit_identical():
    """ancillary 를 안 주면 Biased-MPPI 이전과 비트 단위로 같다. 세 가지 '없음' 표현
    모두 같은 경로를 타야 한다 -- None 만 검사하면 빈 배열이 다른 난수열을 쓰는 것을
    놓친다."""
    for anc in (None, [], np.zeros((0, 25, 3))):
        m, x, wp = _biased()
        for want in STEP_U:
            u = m.step(x, wp, goal_theta=0.3, ancillary=anc)
            np.testing.assert_array_equal(u, want)
            x = x + u * m.dt
        assert m._groups == (m.K,)
        assert m.last_w_groups == (1.0,) or abs(m.last_w_groups[0] - 1.0) < 1e-9


def test_ancillary_samples_are_limited():
    """ancillary 도 같은 _limit 을 거친다 -- 한계를 한참 넘는 열을 줘도 롤아웃의 첫
    스텝 속도는 디스크 안이다."""
    m, x, wp = _biased()
    m.step(x, wp, ancillary=np.full((3, m.T, 3), 9.0))
    v = (m.last_X[:, 0, :2] - x[:2]) / m.dt          # X 는 x_1.. 이므로 첫 변위 = V[:,0]*dt
    assert np.all(np.linalg.norm(v, axis=1) <= m.v_max[0] + 1e-9)


def test_group_weights_and_is_means():
    m, x, wp = _biased()
    m.step(x, wp, ancillary=np.zeros((3, m.T, 3)))
    assert m._groups == (449, 21, 21, 21), m._groups          # 512 - 3*21
    assert len(m.last_w_groups) == 4 and len(m.last_is_groups) == 4
    assert abs(sum(m.last_w_groups) - 1.0) < 1e-9, m.last_w_groups
    assert all(np.isfinite(v) for v in m.last_is_groups)


def test_a_good_ancillary_wins_the_weight():
    """캐럿으로 곧장 가는 열, 반대로 가는 열, 직각으로 가는 열을 주면 첫 블록이 이긴다."""
    m, x, wp = _biased()
    d = wp - x[:2]
    d = d / np.linalg.norm(d) * m.v_max[0]
    anc = np.zeros((3, m.T, 3))
    anc[0, :, :2] = d                                        # 좋은 후보
    anc[1, :, :2] = -d                                       # 반대
    anc[2, :, :2] = (-d[1], d[0])                            # 직각
    for _ in range(3):
        x = x + m.step(x, wp, ancillary=anc) * m.dt
    assert m.last_w_groups[1] > m.last_w_groups[2], m.last_w_groups
    assert m.last_w_groups[1] > m.last_w_groups[3], m.last_w_groups


def test_too_many_ancillary_copies_is_refused():
    """가우시안 블록이 절반 밑으로 내려가면 조용히 돌지 않고 터진다."""
    m = MPPI(_empty(), MPPIConfig(K=64, eta_min=3.2, eta_max=6.4, anc_copies=21))
    try:
        m.step(np.zeros(3), np.array([1.0, 0.0]), ancillary=np.zeros((3, m.T, 3)))
    except AssertionError:
        return
    raise AssertionError('가우시안 샘플이 1 개 남는 배분이 통과했다')


def test_ancillary_shape_is_checked():
    m, x, wp = _biased()
    try:
        m.step(x, wp, ancillary=np.zeros((3, m.T + 1, 3)))
    except AssertionError:
        return
    raise AssertionError('지평선 길이가 다른 ancillary 가 통과했다')


# ── obs_ref: 목표 셀 기준 단측 벌점 ──────────────────────────────────────────
def test_obs_ref_zero_is_bit_identical():
    """obs_ref=0.0 이면 기존 골든과 비트 동일. 비용은 항상 >= 0 이므로
    max(0, c - 0.0) == c 가 정확히 성립한다. 기본값 생략도 같은 경로여야 한다."""
    m, X, V = _golden()
    np.testing.assert_array_equal(m._cost(X, V, _G_WP, _G_THETA, None, 0.0), S_NONE)
    np.testing.assert_array_equal(m._cost(X, V, _G_WP, _G_THETA, None), S_NONE)
    np.testing.assert_array_equal(
        m._cost(X, V, _G_WP, _G_THETA, _golden_volume(m), 0.0), S_DYN)


def test_obs_ref_discounts_only_above_the_baseline():
    """기준선 아래는 0, 위는 (c - ref) 로만 센다. 깎이기만 하고 늘지 않는다."""
    m, X, V = _golden()
    c = m.grid.lookup(X[..., :2])
    ref = float(np.median(c))
    m._cost(X, V, _G_WP, _G_THETA, None, 0.0)
    obs0 = m.last_terms['obs'].copy()
    m._cost(X, V, _G_WP, _G_THETA, None, ref)
    want = (m.w_obs * np.maximum(0.0, c - ref) * m.disc).sum(axis=1)
    assert np.allclose(m.last_terms['obs'], want)
    assert np.all(m.last_terms['obs'] <= obs0 + 1e-9)
    assert np.any(m.last_terms['obs'] < obs0 - 1e-9)


def test_obs_ref_does_not_soften_lethal_rejection():
    """기준선을 띠의 최댓값까지 올려도 lethal 롤아웃은 여전히 탈락한다.

    _empty() 는 장애물이 없어 n_hit 이 항상 0 이라 이 검사가 공허해진다 -- 그래서
    실제 벽이 있는 scene 격자를 쓰고, 롤아웃이 닿는 거리에 벽을 두고 n_hit > 0 을
    함께 못 박는다."""
    from nav.grid import static_grid
    g = static_grid()
    # 문틈 벽(x = 3.0~3.3) 바로 앞. 지평선 2.5 s x 0.5 m/s = 1.25 m 안에 벽이 있다.
    x = np.array([2.60, 2.00, 0.0])
    wp = np.array([4.00, 2.00])                  # 벽 건너편 -- 직진 롤아웃은 벽을 읽는다
    hits = []
    for ref in (0.0, 1.0):
        m = MPPI(g, MPPIConfig(seed=2))
        m.step(x, wp, obs_ref=ref)
        hits.append(m.last_n_hit)
    assert hits[0] > 0, f'벽이 안 잡혔다 -- 검사가 공허하다 (n_hit={hits[0]})'
    assert hits[0] == hits[1], f'obs_ref 가 hit 판정을 바꿨다: {hits}'
