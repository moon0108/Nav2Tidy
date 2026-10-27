# 이 코드베이스의 MPPI — 수식과 코드 대응

참고 문헌: Williams, Wagener, Goldfain, Drews, Rehg, Boots, Theodorou,
*Information-Theoretic MPC for Model-Based Reinforcement Learning* (ICRA 2017) 및 T-RO 2018.
표기는 그 규약을 따른다: 샘플 인덱스 $k$는 위첨자, 시간 인덱스 $t$는 아래첨자, 명목 입력은 $U$,
샘플된 입력은 $V$.

다른 파일을 명시하지 않은 코드는 모두 `nav/mppi.py`이고, 인용한 코드 조각은 주석을 덜어낸 것이다.
줄 번호는 현재 작업 트리 기준이다.

---

## 1. 기호 대응표

| 수식 | 코드 | shape | 기본값 |
|---|---|---|---|
| $x_t = (x, y, \theta)$ | `x0`, `X[k, t]` | `(3,)`, `(K, T, 3)` | — |
| $u_t = (v_x, v_y, \omega)$ | `U[t]` | `(T, 3)` | 0 |
| $\varepsilon^k_t$ | `E[k, t]` | `(K, T, 3)` | — |
| $v^k_t = u_t + \varepsilon^k_t$ | `V[k, t]` | `(K, T, 3)` | — |
| $\Sigma = \mathrm{diag}(\sigma^2)$ | `sigma ** 2` | `(3,)` | `(0.35, 0.35, 0.60)` |
| $\lambda$ (온도) | `lam` | 스칼라 | `0.5` |
| $\gamma$ (할인율) | `gamma`, `disc` | 스칼라, `(T,)` | `0.98` |
| $K$ (샘플 수) | `K` | — | `512` |
| $T$ (지평선 스텝) | `T` | — | `25` → 2.5 s |
| $\Delta t$ | `dt` | — | `0.1` s |
| $S^k$ (샘플 총비용) | `S` | `(K,)` | — |
| $w^k$ (가중치) | `w` | `(K,)` | — |
| $q(x, t)$ (단계별 비용) | `_cost` | — | — |

입력 한계 `v_max = (0.5, 0.5, 1.0)`은 `base.xml`의 액추에이터 한계에서 가져왔다. 계획 단계에서
실기가 실행할 수 없는 움직임을 애초에 요구하지 못하게 하려는 것이다.

---

## 2. 동역학: 순수 적분기, 그리고 그것이 왜 *정확*한가

TidyBot++ 베이스는 파워드 캐스터 홀로노믹이라 **글로벌** 프레임에서 속도 명령이 그대로 적분된다.

$$x_{t+1} = x_t + u_t \, \Delta t$$

한 제어주기 안에서 명령 속도는 일정하므로, 이 Euler 스텝은 근사가 아니라 $\dot{x} = u$의 해석해다.
그래서 롤아웃이 `cumsum` 한 번으로 끝나고 **이산화 오차가 0**이다.

```python
def _rollout(self, x0, V):
    """V: (K, T, 3) -> X: (K, T, 3). Pure integrator, zero discretisation error."""
    return x0 + np.cumsum(V * self.dt, axis=1)          # mppi.py:119-125
```

롤아웃 안에서 MuJoCo를 스텝하지 않는다. MuJoCo의 역할은 ground truth 월드와 시각화뿐이다.
실제 베이스에는 캐스터 지연과 가속도 한계가 있는데, 그건 플래너 *바깥*에 있다 — 명령 포즈를
액추에이터 목표로 바꾸는 Ruckig OTG(`mujoco_env.py`의 `BaseController`)가 담당한다. 즉 플래너는
약간 이상화된 모델 위에서 최적화하고, 실행 가능성은 OTG가 만들어 준다.

---

## 3. 최적화 문제

MPPI가 푸는 것은 다음이다.

$$\min_{U} \; \mathbb{E}\left[ \phi(x_T) + \sum_{t=0}^{T-1} \left( q(x_t) + \tfrac{\lambda}{2} u_t^\top \Sigma^{-1} u_t \right) \right]$$

$u^\top \Sigma^{-1} u$ 항은 임의로 덧붙인 제어 노력 페널티가 아니라 KL 제어 유도에서 자연히
나오는 항이다.

### 정보이론적 결과

기울기를 계산하는 대신, 유도는 **최적 제어분포** $q^*$를 직접 지목한다. 제어되지 않은 기저분포
$p$에 대한 밀도비는 다음과 같다.

$$\frac{\mathrm{d}q^*}{\mathrm{d}p}(V) = \frac{1}{\eta} \exp\left(-\tfrac{1}{\lambda} S(V)\right), \qquad \eta = \mathbb{E}_p\left[\exp\left(-\tfrac{1}{\lambda} S(V)\right)\right]$$

최적 입력열은 $q^*$의 평균이다. $q^*$에서 직접 샘플링할 수 없으므로 현재 제안분포
$\mathcal{N}(U, \Sigma)$에서 뽑고 importance sampling으로 재가중한다. $q^*$와 제안분포 사이의
likelihood ratio가 추가 제어항들을 만들어 내고, 실제로 쓰이는 가중치는 이렇게 된다.

$$w^k = \frac{1}{\eta} \exp\left(-\frac{1}{\lambda}\left[ S(V^k) + \lambda \sum_{t=0}^{T-1} u_t^\top \Sigma^{-1} \varepsilon^k_t + \frac{\lambda}{2}\sum_t u_t^\top \Sigma^{-1} u_t + \frac{\lambda(1 - \nu^{-1})}{2}\sum_t \varepsilon_t^{k\top} \Sigma^{-1} \varepsilon^k_t \right]\right)$$

이 네 항 중 둘이 이 구현에서 사라지는데, 이유를 이해해 둘 값이 있다.

- $\frac{\lambda}{2}\sum_t u_t^\top \Sigma^{-1} u_t$ 는 $k$에 의존하지 않는다. softmax 안의 상수는
  정규화 항과 상쇄되므로 빼도 결과가 완전히 같다.
- $\frac{\lambda(1-\nu^{-1})}{2}\sum_t \varepsilon^\top \Sigma^{-1}\varepsilon$ 는 $\nu = 1$일 때
  계수가 0이다. 즉 샘플링 공분산과 제어 공분산이 같을 때다. 여기서는 탐색 공분산을 따로 부풀리지
  않으므로 $\nu = 1$이고 이 항은 없다.

남는 것이 정확히 코드가 계산하는 식이다.

$$S^k = \underbrace{\sum_{t=0}^{T-1} \gamma^t q(x^k_t, t)}_{\text{상태 비용}} + \underbrace{\lambda \sum_{t=0}^{T-1} u_t^\top \Sigma^{-1} \varepsilon^k_t}_{\text{importance sampling 항}}$$

$$w^k = \frac{\exp\left(-\frac{1}{\lambda}(S^k - \beta)\right)}{\sum_j \exp\left(-\frac{1}{\lambda}(S^j - \beta)\right)}, \qquad \beta = \min_j S^j$$

$$U \leftarrow U + \sum_{k=1}^{K} w^k \varepsilon^k$$

여기서 $\lambda$가 IS 항의 계수와 지수의 온도 양쪽에 등장하므로 그 항에서는 서로 상쇄되어
지수에는 $-\sum_t u_t^\top \Sigma^{-1}\varepsilon_t$ 만 남는다. 우연히 맞아떨어진 게 아니라 유도의
구조 자체다.

---

## 4. 단계별 비용 $q$

$$q(x, u, t) = w_{\text{goal}} \lVert p - p_{\text{wp}} \rVert + w_{\text{obs}} \, c_{\text{static}}(p) + w_{\text{dyn}} \, c_{\text{dyn}}(t, p) + w_{\text{head}} \lvert \Delta\theta \rvert + q_u(u_t, u_{t-1})$$

$p = (x, y)$이고 $\Delta\theta$는 $[-\pi, \pi)$로 감싼 값이다.

```python
def _cost(self, X, V, wp, goal_theta, cost_volume):     # mppi.py:155-187
    p = X[..., :2]                                      # (K, T, 2)
    q = self.w_goal * np.linalg.norm(p - wp, axis=-1)
    q += self.w_obs * self.grid.lookup(p)
    if cost_volume is not None:
        q += self.w_dyn * self._dyn_lookup(p, cost_volume)
    dth = (X[..., 2] - goal_theta + np.pi) % (2 * np.pi) - np.pi
    q += self.w_head * np.abs(dth)
    q += self._control_cost(V)
    return (q * self.disc).sum(axis=1)                  # (K,)
```

가중치: $w_{\text{goal}} = 15$, $w_{\text{obs}} = 20$, $w_{\text{dyn}} = 60$,
$w_{\text{head}} = 0.5$.

### 제어 비용 $q_u$ (`_control_cost`)

앞의 네 항은 롤아웃된 *상태* $X$에 걸리지만, 이 항만은 *입력* $V$에 직접 걸린다.

$$q_u(u_t, u_{t-1}) = \tfrac{1}{2} u_t^\top R \, u_t + \tfrac{1}{2} (u_t - u_{t-1})^\top R_d (u_t - u_{t-1}) + w_{\text{dir}} \, c_{\text{dir}}(u_t, u_{t-1})$$

$$c_{\text{dir}}(u_t, u_{t-1}) = (1 - \cos\Delta\psi_t) \cdot \lVert (v_{x,t}, v_{y,t}) \rVert, \qquad \cos\Delta\psi_t = \frac{v_t \cdot v_{t-1}}{\lVert v_t \rVert \lVert v_{t-1} \rVert}$$

- 1항은 입력 크기($R$), 2항은 입력 변화율($R_d$) — 각각 급격한 명령과 급격한 변화를 억제한다.
- 3항은 $v_x, v_y$ 성분만으로 방향 변화를 재고 거기에 속력을 곱한다. 회전 $\omega$는 제자리
  선회라 진행 방향과 무관하므로 제외했고, 속력을 곱하는 것은 저속에서 방향 페널티가 과해지지
  않게 하려는 것이다. $\lVert v_t \rVert$나 $\lVert v_{t-1} \rVert$이 0이면 사잇각이 정의되지
  않으므로 $\cos\Delta\psi = 1$로 두어 항을 0으로 만든다 — 그러지 않으면 정지 상태에서 출발할
  때마다 있지도 않은 방향 전환에 벌점이 붙는다.

$R = \mathrm{diag}(r_v, r_v, r_w)$, $R_d = \mathrm{diag}(r_{dv}, r_{dv}, r_{dw})$로 대각이고,
설정 필드는 전부 스칼라다 (`sweep.py`가 튜플 필드는 훑지 못한다). $t = 0$에서의 $u_{t-1}$은
직전 solve가 실제로 실행한 명령(`self.u_prev`)이라, 페널티가 제어주기 경계를 넘어 이어진다.

기본값은 다섯 손잡이 모두 $0$이다. 즉 항 전체가 정확히 0이 되어, 이 항이 생기기 전에 기록해 둔
벤치마크와 그대로 비교된다.

### 정적 장애물 (`nav/grid.py`의 `Grid.bake`)

점유격자의 Euclidean distance transform으로 가장 가까운 벽까지의 거리 $d(p)$를 얻고, 비용은 그
거리에 대한 지수함수로 두되 로봇 반경 $r = 0.35$ m를 기준점으로 잡는다.

$$c_{\text{static}}(p) = \min\left(\exp\left(-\frac{d(p) - r}{\text{decay}}\right),\ 10\right), \qquad d \le r \text{ 인 곳은 } 10$$

`decay = 0.15` m로 짧게 잡은 것은 의도적이다. PLAN §7에 `decay = 0.4`로 두면 장애물 비용이 자유
공간까지 새어 나가 로봇이 어디서든 굼떠진다는 기록이 있다.

### 동적 장애물 (`nav/cost_volume.py`)

보행자 상태를 등속도로 외삽하고, 각 예측 위치에 같은 지수 형태를 찍는다. 다만 기준점이 로봇 반경이
아니라 **접촉 거리** $r_c = r_{\text{robot}} + r_{\text{ped}} = 0.65$ m다.

$$c_{\text{dyn}}(t, p) = \max_{i} \; \min\left(\exp\left(-\frac{\lVert p - (p_i + v_i \tau) \rVert - r_c(\tau)}{\text{decay}(\tau)}\right),\ 1\right) \cdot \kappa^{\tau/\Delta t}$$

$\tau = t\Delta t + a$ 는 예측 리드 시간이고 $a$는 스냅샷의 나이(§7 참조), $r_c(\tau) = 0.65 +
0.05\tau$, $\text{decay}(\tau) = 0.3 + 0.1\tau$ 로 리드가 길수록 불확실성이 커진다. $\kappa$는
스텝별 예측 신뢰도(`confidence_decay`, 기본 1.0 = 할인 없음)다.

보행자들은 $\sum$이 아니라 $\max$로 합성한다. 그래야 군중 밀도와 무관하게 볼륨이 $[0, 1]$에
머무는데, 이건 나중에 이 모듈을 대체할 실제 탐지/예측 모듈과의 인터페이스 계약이다.

조회는 볼륨의 첫 축을 지평선 스텝으로 인덱싱한다. 그래서 `p`의 shape이 반드시 `(K, T, 2)`여야
한다.

```python
def _dyn_lookup(self, p, cv):                           # mppi.py:189-204
    idx = np.floor((p - self.grid.origin) / self.grid.res).astype(np.int32)
    n, H, W = cv.shape
    c = np.clip(idx[..., 0], 0, W - 1)
    r = np.clip(idx[..., 1], 0, H - 1)
    t = np.clip(np.arange(self.T), 0, n - 1)[None, :]
    return cv[t, r, c]
```

---

## 5. `step()` 한 번, 한 줄씩

```python
def step(self, x0, wp, goal_theta=0.0, cost_volume=None):        # mppi.py:206
    E = self.rng.normal(0.0, self.sigma, size=(self.K, self.T, 3))   # (1)
    V = np.clip(self.U + E, -self.v_max, self.v_max)                 # (2)
    E = V - self.U                                                   # (3)

    X = self._rollout(x0, V)                                         # (4)
    S = self._cost(X, wp, goal_theta, cost_volume)                   # (5)
    S += self.lam * np.einsum('td,ktd,d->k', self.U, E, 1.0 / self.sigma ** 2)   # (6)

    w = np.exp(-(S - S.min()) / self.lam)                            # (7)
    w /= w.sum()
    self.U += np.einsum('k,ktd->td', w, E)                           # (8)
    self.U = np.clip(self.U, -self.v_max, self.v_max)

    u = self.U[0].copy()                                             # (9)
    self.U = np.roll(self.U, -1, axis=0)
    self.U[-1] = 0.0
    return u
```

1. $\varepsilon^k \sim \mathcal{N}(0, \Sigma)$ 를 $K$개 샘플 × $T$ 스텝 전부 한 번에 뽑는다.
   `rng`는 인스턴스별 `default_rng(seed)`다 — 전역 시드를 쓰지 않으므로 에피소드가 재현된다.
2. $v^k_t = \mathrm{clip}(u_t + \varepsilon^k_t)$. 클리핑이 샘플을 액추에이터 한계 안에 붙잡아 둔다.
3. **클리핑 후 노이즈를 다시 정의한다.** 갱신식에서 논문과 다른 유일한 지점이고, 중요하다.
   (6)의 IS 항은 $\varepsilon$가 *실제로 적용된* 섭동일 때만 옳다. 클리핑이 그걸 바꿔 버리므로
   $\varepsilon \leftarrow v - u$로 일관성을 회복시킨다.
4. $X^k = x_0 + \mathrm{cumsum}(V^k \Delta t)$ — $K \times T$개 상태를 벡터화 한 번으로.
5. 상태 비용을 지평선에 걸쳐 할인 합산.
6. IS 항. `einsum('td,ktd,d->k', U, E, 1/sigma**2)`는 대각 $\Sigma$에 대해
   $\sum_t \sum_d u_{td}\,\varepsilon^k_{td}/\sigma_d^2 = \sum_t u_t^\top \Sigma^{-1}\varepsilon^k_t$
   와 문자 그대로 같다.
7. $\beta = \min_k S^k$ 를 뺀 softmax — §6 참조.
8. $U \leftarrow U + \sum_k w^k \varepsilon^k$, 그리고 명목 입력 자체도 실행 가능하도록 클리핑.
9. Receding horizon: $u_0$를 실행하고 열을 한 칸 앞으로 밀고 새 말단을 0으로 채운다. 이 shift가
   warm start이고, 그래서 $K = 512$ 샘플로 충분하다 — 매 solve가 이전 계획을 이어 다듬는다.

측정값: $K = 512$, $T = 25$에서 평균 1.3 ms, 최대 1.9 ms (순수 numpy, GPU 없음).

---

## 6. 몸에 익혀 둘 수치 두 가지

### $\beta$ subtraction은 선택이 아니다

softmax는 입력에 상수를 더해도 값이 같다: $\mathrm{softmax}(S)_k = \mathrm{softmax}(S - \beta)_k$.
이 shift가 없으면 $S \sim 10^3$, $\lambda = 0.5$에서 $\exp(-2000) = 0$ (float64 underflow)이 되어
모든 가중치가 0이 되고, `w /= w.sum()`이 0으로 나누며 $U$가 `NaN`이 된다. $\min_k S^k$를 빼면 가장
큰 가중치가 정확히 $\exp(0) = 1$이 되도록 보장된다.

`nav/tests/test_mppi.py::test_beta_subtraction_survives_huge_costs`가 $w_{\text{goal}} = 10^5$
($S$가 $10^6$까지 커지는 조건)로 이걸 고정해 둔다.

### Effective sample size: 이 MPPI는 사실상 greedy다

$$K_{\text{eff}} = \frac{1}{\sum_k (w^k)^2}$$

$K_{\text{eff}} = K$면 균등 가중, $K_{\text{eff}} = 1$이면 한 샘플이 전부를 가져간다는 뜻이다.
여기서 측정값은 **$K_{\text{eff}} \approx 1.0$**. $\lambda = 0.5$인데 샘플 간 비용 차이가 수십
단위라 softmax가 포화되고, 그래서 갱신은 실질적으로 *좋은 것들을 평균내기*가 아니라 *가장 좋은
롤아웃 고르기*로 축약된다.

이건 수식의 오류가 아니라 튜닝의 결과다. 그리고 `nav/viz.py`가 가중치 상위 샘플이 아니라 샘플
구름의 균등 슬라이스를 그리는 이유이기도 하다 — "상위 가중치"로 뽑으면 의미 있는 롤아웃 1개와
나머지 $n-1$개의 임의 롤아웃이 나온다. $\lambda$를 키우면 평균화 거동이 돌아온다.

```bash
python -m nav.sweep --mppi lam=0.5,2,5 --densities 4 --seeds 8
```

PLAN §7이 $\lambda = 0.5$를 검증값으로 고정해 두었으므로 기본값은 그대로 두었다.

스윕할 때 함정 하나: **보행자 2명 · 시드 0은 알려진 livelock 에피소드다** (로봇이 목표 3 m 앞에서
90초 내내 맴돈다 — `nav/NOTES.md` Phase 4 참조). 뷰어로 관찰하기엔 좋은 사례지만 파라미터를
판단하는 근거로는 나쁘다. `--seeds 1`이 아니라 `--seeds 8` 이상을 쓸 것.

---

## 7. 이 구현이 논문과 다른 곳

| 논문 | 이 코드 | 이유 |
|---|---|---|
| 무할인 유한지평선 + 말단비용 $\phi(x_T)$ | $\gamma^t$ 할인, 말단비용 없음 | PLAN §3.3 사양. 목표항을 $T$에서만이 아니라 매 스텝 넣는다. 할인은 상태비용에만 걸리고 IS 항에는 걸리지 않는다. |
| 명시적 입력 클리핑 없음 | $V$를 클리핑하고 $\varepsilon \leftarrow V - U$ | 액추에이터 한계는 실재한다. $\varepsilon$를 다시 구해야 IS 항이 정직해진다. |
| $\nu > 1$ 선택 가능 (탐색 공분산 부풀리기) | $\nu = 1$ | 필요 없다. $\varepsilon^\top\Sigma^{-1}\varepsilon$ 항이 사라진다. |
| $U$에 Savitzky-Golay 스무딩 | 있음 (`sgf_window=9`, `sgf_order=2`, 기본 켜짐) | 평균 $\lvert\text{jerk}\rvert$가 평탄했던 것은 매끄러웠기 때문이 아니라 `Metrics`가 OTG *하류*의 베이스 포즈를 차분하기 때문이었다. 명령 $u$를 직접 재면 스텝 간 2차 차분이 0.73이고, 이 필터가 그것을 0.38로 줄인다. `nav/NOTES.md`의 흔들림 절 참고. |
| 레이턴시는 암묵적 | `_dyn_lookup`이 아니라 코스트 볼륨에서 처리 | 나이 $a$인 스냅샷으로 만든 볼륨이 스스로 시간축을 $a$만큼 시프트한다. 그러면 인덱스 $t$는 언제나 "지금부터 $t$ 제어주기 후"를 뜻하고 MPPI 쪽은 손대지 않아도 된다. `--latency 0.15`로 지연을 주입한다. |

---

## 8. 전체 제어 루프에서 MPPI의 자리

```
A*  (nav/astar.py, 0.10 m 격자, 1 Hz 또는 경로에서 1 m 이상 벗어날 때 재계획)
      |  path: (N, 2)
lookahead(path, x, 0.75 m)
      |  wp: (2,)
코스트 볼륨 (nav/cost_volume.py, 보행자 상태로부터)
      |  cv: (T, H, W), 값은 [0, 1]
MPPI.step(x, wp, goal_theta, cv)  ->  u = (vx, vy, omega)
      |
env.step({'base_pose': x + u * 1.5})     # 1.5 s 리드, gamepad_teleop.py:71 관용구
      |
BaseController: Ruckig OTG -> position 액추에이터 -> MuJoCo, 10 Hz
```

1.5 s 리드는 틀리기 쉬운 지점이라 이해해 둘 필요가 있다. OTG는 주어진 포즈에서 *정지*하도록
계획하므로, $x + u\Delta t$(5 cm 앞)를 명령하면 베이스가 0.14 m/s에 묶인다. $x + 1.5u$를 주면
목표가 충분히 멀어져서, 속도를 제한하는 것이 목표까지의 거리가 아니라 OTG 자신의 가속도 한계가
된다.

---

## 9. 뷰어에서 읽는 법

```bash
python -m nav.runner --peds 4 --ped-policy sfm --cost-volume
```

- 초록 — A* 전역 경로
- 노란 구 — lookahead 웨이포인트 $p_{\text{wp}}$, MPPI가 보는 유일한 목표
- 청록 — 갱신 후의 명목 계획 $U$, 즉 실제로 실행될 궤적
- 주황 — 샘플 롤아웃 $X^k$ (`--viz-samples N`)

샘플 부채꼴이 짧은 것은 설계상 정상이다. $T\Delta t = 2.5$ s에 0.5 m/s면 약 1 m이고, 노이즈
random walk가 대략 $\sigma \Delta t \sqrt{T} \approx 0.18$ m의 퍼짐을 더한다. 줌인해서 볼 것.

---

## 10. 파라미터를 몸으로 느껴 보는 실습

| 변경 | 예상되는 결과 |
|---|---|
| `lam=5` | 가중치가 퍼져 $K_{\text{eff}} \gg 1$, 더 매끄럽지만 결단력은 떨어진다 |
| `w_goal=5` | 문틈을 통과하지 못한다 — PLAN §7에 기록된 실패 모드 |
| `gamma=1.0` | 할인 없음. 먼 미래 비용이 즉각 비용과 같은 무게를 가져 더 보수적이 된다 |
| `T=10` | 보행자를 돌아갈 만큼 멀리 못 본다. 충돌 증가 |
| `K=64` | solve는 싸지고 계획은 거칠어진다. warm start가 상당 부분 가려 준다 |
| `sigma=(0.1, 0.1, 0.2)` | 탐색이 좁아 현재 계획을 고집하고 회피가 늦어진다 |

전부 플래그 하나 거리다.

```bash
python -m nav.sweep --mppi lam=0.5,5 T=10,25 --densities 4 --seeds 8
```

---

## 11. Biased sampling (ancillary 제어열)

Biased-MPPI (arXiv:2401.09241). 가우시안 샘플 일부를 **의도된 제어열**로 대체한다.
열 자체는 `step()` 이 `ancillary: (A, T, 3)` 로 **받는다** — 생성은 경로를 아는 쪽
(`scripts/executor.py` 의 `_ancillary()`)의 일이고, MPPI 는 끝까지 경로를 모른다.
`nav/runner.py` 는 넘기지 않으므로 MuJoCo 쪽 수치는 그대로다.

### 샘플 배분 (단계 (1)(2))

$$K = \underbrace{K_g}_{449} + \underbrace{A}_{3} \times \underbrace{c}_{21}$$

- 가우시안 블록: $V^k = \mathrm{limit}(\bar U + \varepsilon^k)$ — 예전 그대로.
- ancillary 블록 $a$: 사본 0 은 $\mathrm{limit}(\mathrm{anc}_a)$ 로 **노이즈가 없고**
  (의도한 기동 그 자체가 반드시 후보에 있어야 한다), 사본 1..c-1 은
  $\mathrm{limit}(\mathrm{anc}_a + \mathcal N(0, (0.25\,\sigma)^2))$.
- `assert` $K_g \ge K/2$ — `K` 를 줄이면서 `anc_copies` 를 그대로 두면 가우시안
  블록이 급격히 작아진다. `eta` 밴드가 `K` 를 따라가지 않는 것과 같은 함정이다.
- $A = 0$ 이면 난수 호출의 모양과 순서까지 예전과 같아 **결과가 비트 동일**하다
  (`test_ancillary_none_is_bit_identical` 이 골든으로 고정).

### IS 항 (단계 (6))

$\varepsilon = V - \bar U$ 를 ancillary 에도 **그대로** 쓴다. 그 샘플은 $\bar U + \varepsilon$
꼴이 아니므로 $|\varepsilon|$ 이 커지고, $I = \sum_t \bar U_t^\top \Sigma^{-1}\varepsilon_t$ 가
큰 고정 벌점이 된다. 부호는 직관과 반대다 — $\bar U$ 와 **같은** 방향인 열이
$I > 0$ 으로 벌점이고, 반대 방향이 보상이다.

실측 (2026-10-06, 시뮬 team 모드): `is_path` $\approx +19.6$, `is_gauss` $\approx -1.6$.
$\lambda \approx 7$ 이므로 샘플당 가중치 비 $e^{-21/7} \approx 0.05$, 전체 $\Sigma w$ 는
$63 \cdot 0.05 / (449 + 63\cdot0.05) \approx 0.006$ — 측정값과 일치한다.

**다만 치명적이지는 않다.** 상태비용 우위가 더 크면 이긴다 (자유 공간 실측:
`path` 블록이 상태비용 $-32.4$, IS $+24.2$ → 샘플당 가우시안의 4.5 배, $\Sigma w = 0.173$).
명목해가 이미 좋은 추종해일 때는 ancillary 가 새로 줄 것이 없어 IS 벌점만 남아 진다.
그것이 Biased-MPPI 의 정상 동작이다 — 효과는 **막혔을 때** 나타난다.

### 갱신식은 바꾸지 않았다 (단계 (8))

$\bar U \leftarrow \bar U + \sum_k w^k \varepsilon^k$ 는 $\varepsilon = V - \bar U$ 와
$\sum_k w^k = 1$ 때문에 **이미** sample-average 다:

$$\bar U + \sum_k w^k (V^k - \bar U) = \sum_k w^k V^k - \bar U \sum_k w^k + \bar U = \sum_k w^k V^k$$

$\bar U + \varepsilon$ 꼴이 아닌 샘플에도 성립한다. 필요한 것은 저 두 사실뿐이고
둘 다 무조건 성립한다 (탈락 롤아웃은 $w = 0$). 실측: 가중치를 ancillary 블록에만
몰면 갱신이 정확히 그 블록의 평균이 되고, 두 식의 차이는 $1.3\times10^{-15}$
($\bar U$ 스케일 대비 $2.9\times10^{-15}$) 로 반올림뿐이다. **$\sum w V$ 로 고쳐 쓰면
그 $10^{-15}$ 때문에 "ancillary 0 개면 비트 동일" 불변 조건이 깨진다.**

### 진단

`last_w_groups`, `last_is_groups` (둘 다 길이 $1+A$). 블록별 $\Sigma w$ 와
$\texttt{is\_coeff}\cdot I$ 평균이다. 두 질문이 다르다 — 전자는 "어느 후보가
이겼는가", 후자는 "IS 항이 그 블록을 체계적으로 밀어내는가". `last_w_bins`
(방향 4 분할)과도 직교한다: 저쪽은 "어디로 가려는가"다.

### 실측과 사고 기록 (2026-10-07)

#### ancillary 가 코너를 깎아 전원 탈락했다

처음 구현은 경로 추종 열을 **매 스텝 예측 위치에서 lookahead 점을 다시 잡아 그쪽으로
직선 진행**하게 만들었다. 그러면 경로의 곡률을 가로질러 코너를 깎는다. 증상은
"biasing 을 켜도 아무 변화가 없다"였고, 블록별 $\Sigma w$ 만 보면 path/side 가 0.000
이라 *비용이 높아 졌다* 로 읽힌다. 실제로는 **21 개가 전원 lethal 마스킹**이었다.

단서는 하나뿐이었다: `n_hit` 이 biased off 2 $\to$ on **65** (= 블록 크기 $3\times21=63$).
그래서 `last_hit_groups` / CSV `hit_*` 4 열을 먼저 넣었고, 그 다음 정체 시점의
라이브 $\max(\text{static},\text{local})$ grid 스냅샷으로 재현해 확정했다
(지령 0.0101 m/s, 실측 0.012~0.015 와 일치):

| 검사 | lethal | 최대 비용 |
|---|---|---|
| 경로 자체 | — | 0.500 |
| 로봇 $\to$ 캐럿 직선 | 0/40 | 0.602 |
| **`path` ancillary 롤아웃** | **7/25** | **10.000** |

경로도 직선도 막히지 않는데 롤아웃만 기둥 0.30 m 안에 들어갔다 (inscribed 0.37).

**수정**: 호 길이를 따라 전진한다 (`_at_arc` 보간). 롤아웃이 구성상 경로 위에 놓여
코너를 깎지 않는다. 로봇이 경로에서 벗어나 있으면 첫 스텝이 경로 위로 끌어올린다.
`scripts/test_ancillary.py` 가 이것을 고정한다 -- 라이브 스냅샷으로 찾았지만 정적
지도에서도 재현되므로 (`scene.py` 기하 + `nav.astar`, 경로 곡률 최대 지점에서 옛
방식 lethal 15/25, 호 길이 추종 0/25) costmap 스냅샷을 리포에 넣지 않는다.

같이 있던 버그: `v = v_nom` 은 벡터에 스칼라를 대입한다 (`v = v * (v_nom / n)`).
바로 위 `v = d/dt` 가 최대 $1.5\,v_{cap}$ 까지 나오므로 이 분기는 자주 탄다.

#### 시뮬 A/B 96 에피소드 (6 시나리오 $\times$ 5 설정 $\times$ 3 반복 + heading 재확인)

| 설정 | 도달 | `t_solve` p50 | 정체 max | 접선 yaw | jw p95 / max | `ms` p50 |
|---|---|---|---|---|---|---|
| off | 15/18 | 16.4 s | 6.5 s | 3.3° | 0.67 / 2.15 | 8.4 |
| off-K449 | 15/18 | 16.6 s | 6.3 s | 3.5° | 0.70 / **1.72** | 7.7 |
| **on** | **18/18** | **15.1 s** | **0.0 s** | 2.9° | 0.71 / **3.92** | 10.2 |
| off-wall | 18/18 | 15.5 s | 0.0 s | 4.3° | 0.73 / 2.30 | 8.0 |
| on-wall | 18/18 | 14.9 s | 0.0 s | 3.6° | 0.70 / 3.82 | 11.8 |
| hd-goal | 6/6 | 18.3 s | 0.0 s | **55.3°** | 0.57 / 1.23 | 8.7 |

실패 6 건은 전부 `off` / `off-K449` 의 **clutter** 다. 그것이 명목해가 막히는 유일한
시나리오였다: **off 0/3 (정체 6.5 s) $\to$ on 3/3 (정체 0 s, `t_solve` 14.6 s)**.

**`off-wall` 이 3/3 인 것이 중요하다.** 옛 좁은 inflation (0.40/4.0) 에서는 그 실패가
나지 않는다 -- clutter 정체는 넓은 비용장 (1.05/6.67) 이 만든 것이고 biasing 이 그것을
보상한다. `popin-A/B` 는 어떤 설정에서도 정체하지 않았다 (전부 3/3): 개활 구역의 횡방향
여유가 1.5~2.9 m 라 가우시안 블록만으로 판을 우회한다.

기구는 **간헐적**이다. `w_path` 의 p50 은 0.031 뿐이지만 `w_path > 0.05` 인 tick 이
40 % (59/147), `> 0.10` 이 34 tick, 최대 0.471 이다. 올라가는 시점이 `c_robot` 이 높은
구간과 겹치고, 쉬운 구간에서는 0 으로 내려가며 `w_gauss` 0.99 로 warm start 가 받은
해를 운반한다. 전/후 모드 평균화 (off 의 정체 시 fwd 0.42 / back 0.24) 가 아예
형성되지 않는다 -- on 에서는 `w_fwd` 가 1.000 으로 유지된다.

side 열은 현행 유지한다. 개활 구역에서는 **탈락하지 않는데도** (`hit` 0.00)
$\Sigma w$ 가 0.000 이다 -- 실행 가능하지만 비용에서 지는 것이고 (IS $+24$ 에 목표 항이
직진을 선호), 경로 기준 횡오프셋으로 바꿔도 해결되지 않는다. 보행자 비용 ($w_{dyn}$)
단계에서 재평가한다.

#### ω jerk 스파이크는 **그룹 전환 jerk** 다

`on` 에서만 상위 스파이크가 생긴다 (max 2.15 $\to$ 3.92). 샘플 수가 아니다 -- K 를
512 $\to$ 449 로 줄인 `off-K449` 는 오히려 1.72 로 **내려갔다**. `p95` 는 세 설정이
사실상 같다 (0.67 / 0.70 / 0.71), 즉 체계적 상승이 아니라 드문 사건이다.

$|\Delta w_{path}|$ 와 겹친다 (on + on-wall, 5900 tick):

```
피어슨 r = +0.110            전체로는 약하다 (대부분의 tick 은 둘 다 작다)
jerk >= 1.0:  95 tick,  |Δw_path| > 0.1 비율 24 %  (전체 6 %)
jerk >= 2.0:   8 tick,  |Δw_path| > 0.1 비율 62 %  (전체 6 %)   <- 10 배
              |Δw_path| p50 0.215                (전체 0.010)   <- 21 배
```

최대 jerk tick 의 전이가 `w_path` 0.004 $\to$ 0.369 / 0.008 $\to$ 0.211 처럼 한 tick
도약이고, 그때 `uw` 가 부호까지 바뀐다 (0.085 $\to$ $-$0.298). ancillary 열의 ω 는
$k_{yaw}\,\mathrm{wrap}(yaw_{ref}-\theta)$ 로 가우시안 블록과 무관하게 정해지므로,
블록이 가중치를 가져가는 순간 갱신 $\sum_k w^k V^k$ 의 중심이 한 tick 에 이동한다.
출력 SG 필터가 완화하지만 한 tick 의 큰 이동은 남는다.

**처방 후보 (미적용)**: $R_d$ 의 ω 성분 `rd_w` (현재 0) 를 소량. 스파이크가 정확히
$\Delta u_\omega$ 현상이고, 이 파일 §4 의 실측이 이미 안전 범위를 알려 준다 --
"`rd_w` cuts omega sign flips (48.8 % $\to$ 37.6 % at 2) but from 10 the base wanders".
`anc_sigma_frac` 을 낮추는 것은 겨냥이 틀렸다: 스파이크는 사본의 산포가 아니라 **블록
간 가중치 이동**이고, 0 으로 해도 전환 자체는 남는다.

보류 상태다. `p95` 가 불변이고, 실기는 $v_{max,\omega}=0.6$ 이라 3.92 rad/s² 가 한
제어주기에 낼 수 있는 $\Delta\omega$ 는 0.392 rad/s 로 한계 안에서 포화된다. 실기
on/off 비교에서 문제로 확인되면 그때 실기 프로필 override 로 1.0 / 2.0 을 시험한다.
**그때 집계는 에피소드 시작 1 s 를 제외한다** -- warm start 과도 응답과 그룹 전환
jerk 를 분리해야 한다 (위 표의 utrap 스파이크 세 건이 전부 $t=0.1$ s 다).

#### 측정 오류 정정 내역

이번 세션에서 내가 틀렸던 것들. 같은 함정을 다시 밟지 않기 위해 남긴다.

| 주장 | 왜 틀렸나 |
|---|---|
| "clutter 는 1.0 m 기둥 틈이 하한 1.05 m 아래라서 실패" | 정체 지점의 통로 폭이 **1.27 m** 였다 |
| "clutter 는 목표가 벽에서 0.60 m 라서 실패" | `obs_ref` 가 전 구간 0 이고 rem **4.67 m** 에서 멈췄다 |
| "목표 항 11.25 vs 장애물 11.8 로 맞먹는다" | `c_seg_max` (로봇$\to$캐럿 **직선**) 를 경로 비용으로 읽었다. 실제 항별 비용은 goal 222.3 대 obs 91.8 로 목표 항이 2.4 배 우세다 |
| "경로가 한쪽 벽에 붙어 `cost_travel_multiplier` 부족" | 2.0 / 3.0 / 4.0 이 소수점까지 **같은 경로**를 냈다. 경로 여유 p50 0.640 m = 통로 폭 1.27 m 의 사실상 중앙 |
| "정체 구간 `n_hit` p50 = 2" | 그 CSV 는 `biased=false` 였다. `biased=true` 는 **65** 이고 그 차이가 전원 탈락의 단서였다 |
| "`w_path` 가 낮은데 통과하는 건 한 번 따라간 뒤 warm start 가 이어받아서" | 한 번이 아니라 **40 % 의 tick 에서 반복 개입**한다 |
| "ω jerk 상승은 가우시안 블록이 줄어서" | `off-K449` 가 오히려 내려갔다 (2.15 $\to$ 1.72) |
| 도달 판정을 `rem[-1] < 0.20` 으로 | 정확히 0.20 에 끝난 3 건이 실패로 뒤집혔다. **실행기 자신의 결과**로 판정해야 한다 |
| 결과 파싱 `grep ... \| tail -1` | `timeout` 토큰을 잘못 집어 3 건을 실패로 읽었다. 노드 로그가 권위 있는 출처다 |

측정 하네스에서도 세 번 당했다. (1) 인자 없는 `wait` 가 launch 와 노드까지 기다려
13 시간 멈췄다 -- `wait $!` 로 그 자식만 기다린다. (2) `pkill -f <패턴>` 이 자기
자신의 명령줄에 걸려 셸을 죽였다 -- 측정은 스크립트 **파일**로 돌린다. (3) 측정
스크립트가 패치한 install 트리의 `config/*.yaml` 이 `--symlink-install` 때문에
**소스를 가리키는 심링크**였고, 복원에 쓴 `sed -i` 가 그 심링크를 일반 파일로
갈아치웠다 -- 측정용 설정 변경은 install 트리에 쓰지 않고, 끝나면 `git diff` 를 본다.
