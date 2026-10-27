import numpy as np

from nav import scene
from nav.astar import astar, lookahead
from nav.grid import Grid, plan_grid


def _empty(res=0.05, n=120):
    return Grid(origin=(-1.0, -1.0), res=res, shape=(n, n)).bake(robot_radius=0.35)


def _length(path):
    return np.linalg.norm(np.diff(path, axis=0), axis=1).sum()


def test_optimal_in_empty_space():
    """8-connected A* on an empty grid must return (near) the straight line."""
    g = _empty()
    start, goal = np.array([0.0, 0.0]), np.array([4.0, 3.0])
    path = astar(g, start, goal)
    d = np.abs(goal - start) / g.res
    optimal = g.res * (np.sqrt(2) * d.min() + (d.max() - d.min()))   # 8-connected optimum
    assert path is not None
    assert _length(path) <= optimal + 2 * g.res
    assert np.linalg.norm(path[0] - start) < g.res
    assert np.linalg.norm(path[-1] - goal) < g.res


def test_path_keeps_clearance():
    g = plan_grid()
    path = astar(g, (0.0, 0.0), (8.0, 8.0), clearance=scene.PLAN_CLEARANCE)
    assert path is not None
    d = np.array([g.dist[g.to_cell(x, y)[::-1]] for x, y in path])
    assert d.min() > scene.PLAN_CLEARANCE


def test_path_uses_the_doorway():
    g = plan_grid()
    path = astar(g, (0.0, 0.0), (8.0, 8.0), clearance=scene.PLAN_CLEARANCE)
    crossing = path[np.argmin(np.abs(path[:, 0] - 3.15))]
    assert 3.4 < crossing[1] < 4.6, f'path crossed the wall outside the doorway: {crossing}'


def test_unreachable_returns_none():
    g = Grid.from_boxes([(1.0, -1.0, 1.4, 9.9)], origin=(-1.0, -1.0), res=0.1,
                        shape=(110, 110)).bake(robot_radius=0.35)
    assert astar(g, (0.0, 0.0), (5.0, 5.0)) is None      # wall spans the whole grid


def test_lookahead_advances_along_path():
    path = np.stack([np.linspace(0, 5, 51), np.zeros(51)], axis=1)
    wp = lookahead(path, np.array([1.0, 0.0]), dist=1.5)
    assert np.isclose(wp[0], 2.5, atol=0.15)
    end = lookahead(path, np.array([4.9, 0.0]), dist=1.5)   # clamps at the end
    assert np.allclose(end, path[-1])
