#!/usr/bin/env python3
"""_ancillary 회귀 테스트.  python3 scripts/test_ancillary.py

핵심은 첫 번째 검사다: **경로 추종 열의 롤아웃이 lethal 을 하나도 읽지 않아야 한다.**

이 테스트가 있는 이유. 처음 구현은 매 스텝 예측 위치에서 lookahead 점을 다시 잡아
그쪽으로 직선 진행했다. 그러면 경로의 곡률을 가로질러 **코너를 깎는다**. clutter
시나리오에서 그 깎인 궤적이 기둥 0.30 m 안으로 들어가(inscribed 0.37) 롤아웃이
lethal 로 마스킹됐고, ancillary 21 개가 **전원 탈락**해 Sum(w) = 0.000 이 되었다.
증상은 "biasing 을 켜도 아무 변화가 없다"였고, 블록별 Sum(w) 만 보면 '비용이 높아
졌다'로 읽힌다. n_hit 이 2 -> 65 (= 블록 크기 63) 로 뛴 것이 유일한 단서였다.

라이브 costmap 스냅샷으로 찾았지만 정적 지도에서도 재현된다 -- 경로의 곡률이
가장 큰 지점에서 옛 방식은 lethal 15/25, 호 길이 추종은 0/25 다. 그래서 이 테스트는
스냅샷 없이 scene.py 기하와 nav.astar 만으로 결정적으로 돈다.
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from executor import Executor                   # noqa: E402
from nav import scene                           # noqa: E402
from nav.astar import astar                     # noqa: E402
from nav.grid import static_grid                # noqa: E402
from nav.mppi import LETHAL                     # noqa: E402

T, DT, V_NOM, W_MAX = 25, 0.1, 0.5, 1.0


def _setup():
    """clutter 경로에서 곡률이 가장 큰 지점. 코너 깎기가 일어나는 곳이다."""
    g = static_grid()
    s, gl = scene.SCENARIOS['clutter'][0][:2], scene.SCENARIOS['clutter'][1]
    path = np.asarray(astar(g, s, gl, scene.PLAN_CLEARANCE))
    wrap = lambda a: (a + np.pi) % (2 * np.pi) - np.pi
    best = (-1.0, 5)
    for k in range(5, len(path) - T - 1):
        d1, d2 = path[k] - path[k - 4], path[k + 4] - path[k]
        a = abs(math.degrees(wrap(math.atan2(d2[1], d2[0]) - math.atan2(d1[1], d1[0]))))
        if a > best[0]:
            best = (a, k)
    k = best[1]
    x = np.array([path[k][0], path[k][1],
                  math.atan2(*(path[k + 1] - path[k])[::-1])])
    return g, path, x, best[0]


def _roll(path, arc, x, s_arc, goal):
    """Executor._ancillary 의 경로 추종 열과 같은 적분. 롤아웃 점들을 돌려준다."""
    p, out = x[:2].astype(float).copy(), []
    for _ in range(T):
        v_cap = min(V_NOM, float(np.linalg.norm(goal - p)) / (T * DT))
        s_arc = min(s_arc + v_cap * DT, float(arc[-1]))
        d = Executor._at_arc(path, arc, s_arc) - p
        nd = float(np.linalg.norm(d))
        v = d / DT if nd <= v_cap * DT * 1.5 else v_cap * d / nd
        n = float(np.linalg.norm(v))
        if n > V_NOM:
            v = v * (V_NOM / n)
        p = p + v * DT
        out.append(p.copy())
    return np.array(out)


def test_path_column_never_reads_lethal():
    g, path, x, ang = _setup()
    arc = np.concatenate([[0.0], np.linalg.norm(np.diff(path, axis=0), axis=1).cumsum()])
    i = int(np.argmin(np.linalg.norm(path - x[:2], axis=1)))
    X = _roll(path, arc, x, float(arc[i]), path[-1])
    c = g.lookup(X)
    n_hit = int((c >= LETHAL).sum())
    assert n_hit == 0, (f'경로 추종 롤아웃이 lethal 을 {n_hit}/{T} 스텝 읽었다 '
                        f'(최대 비용 {c.max():.3f}). 코너를 깎고 있다.')
    print(f'  path 열: 꺾임 {ang:.0f} deg 지점에서 lethal 0/{T}, 최대 비용 {c.max():.3f}')


def test_rollout_stays_on_the_path():
    """롤아웃의 모든 점이 경로에서 한 셀 안에 있어야 한다 -- 그것이 '경로 위'의 뜻이다."""
    g, path, x, _ = _setup()
    arc = np.concatenate([[0.0], np.linalg.norm(np.diff(path, axis=0), axis=1).cumsum()])
    i = int(np.argmin(np.linalg.norm(path - x[:2], axis=1)))
    X = _roll(path, arc, x, float(arc[i]), path[-1])
    dev = np.array([np.linalg.norm(path - p, axis=1).min() for p in X])
    assert dev.max() <= scene.GRID_RES * 1.5, f'경로 이탈 최대 {dev.max():.3f} m'
    print(f'  경로 이탈: p50 {np.median(dev):.4f} m, 최대 {dev.max():.4f} m')


def test_at_arc_is_monotone_and_clamped():
    _, path, _, _ = _setup()
    arc = np.concatenate([[0.0], np.linalg.norm(np.diff(path, axis=0), axis=1).cumsum()])
    assert np.allclose(Executor._at_arc(path, arc, -5.0), path[0])
    assert np.allclose(Executor._at_arc(path, arc, float(arc[-1]) + 5.0), path[-1])
    prev = -1.0
    for s in np.linspace(0.0, float(arc[-1]), 60):
        p = Executor._at_arc(path, arc, s)
        k = float(np.linalg.norm(p - path[0]))
        assert k >= prev - 1e-6, f'호 길이 {s:.2f} 에서 역주행'
        prev = min(k, prev if prev > 0 else k)
    print('  _at_arc: 양끝 클램프와 단조성 OK')


if __name__ == '__main__':
    for fn in (test_path_column_never_reads_lethal,
               test_rollout_stays_on_the_path,
               test_at_arc_is_monotone_and_clamped):
        print(fn.__name__)
        fn()
    print('test_ancillary: 전부 통과')
