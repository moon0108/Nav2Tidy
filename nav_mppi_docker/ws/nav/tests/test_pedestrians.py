import numpy as np

from nav import scene
from nav.pedestrians import Crowd, CrowdConfig


def _run(cfg, n=200, robot_xy=None):
    c = Crowd(cfg)
    for _ in range(n):
        c.step(0.1, robot_xy=robot_xy)
    return c


def test_spawn_is_separated_and_clear_of_the_robot():
    c = Crowd(CrowdConfig(n=8, seed=2))
    d = np.linalg.norm(c.pos[:, None, :] - c.pos[None, :, :], axis=2)
    np.fill_diagonal(d, np.inf)
    assert d.min() > 1.0
    assert np.linalg.norm(c.pos - np.asarray(scene.CROWD_START[:2]), axis=1).min() > 1.0


def test_everyone_stays_inside_the_declared_area():
    for policy in ('constant', 'waypoint', 'sfm'):
        c = _run(CrowdConfig(n=8, policy=policy, seed=1))
        x0, y0, x1, y1 = c.cfg.area
        assert c.pos[:, 0].min() >= x0 - 1e-6 and c.pos[:, 0].max() <= x1 + 1e-6, policy
        assert c.pos[:, 1].min() >= y0 - 1e-6 and c.pos[:, 1].max() <= y1 + 1e-6, policy


def test_everyone_actually_walks():
    for policy in ('constant', 'waypoint', 'sfm'):
        c = Crowd(CrowdConfig(n=8, policy=policy, seed=1))
        travelled, prev = np.zeros(8), c.pos.copy()
        for _ in range(100):                 # distance travelled, not net displacement:
            c.step(0.1)                      # the waypoint policy walks a round trip
            travelled += np.linalg.norm(c.pos - prev, axis=1)
            prev = c.pos.copy()
        # Mean, not min: social-force pairs can stand each other off, which is a
        # property of the model rather than a bug.
        assert travelled.mean() > 0.5 * c.cfg.speed * 10.0, policy
        assert np.linalg.norm(c.vel, axis=1).max() <= c.cfg.speed + 1e-6, policy


def test_sfm_keeps_pedestrians_apart():
    c = _run(CrowdConfig(n=8, policy='sfm', seed=4))
    d = np.linalg.norm(c.pos[:, None, :] - c.pos[None, :, :], axis=2)
    np.fill_diagonal(d, np.inf)
    assert d.min() > 2 * c.cfg.radius


def test_sees_robot_changes_the_trajectory():
    """Reciprocity is a flag, not a default: it must actually do something."""
    robot = np.array([6.0, 2.0])
    blind = _run(CrowdConfig(n=8, policy='sfm', sees_robot=False, seed=5), n=50, robot_xy=robot)
    aware = _run(CrowdConfig(n=8, policy='sfm', sees_robot=True, seed=5), n=50, robot_xy=robot)
    assert not np.allclose(blind.pos, aware.pos)
    d_blind = np.linalg.norm(blind.pos - robot, axis=1).min()
    d_aware = np.linalg.norm(aware.pos - robot, axis=1).min()
    assert d_aware > d_blind


def test_crowd_area_must_be_free_of_walls():
    try:
        Crowd(CrowdConfig(area=(2.0, 0.0, 5.0, 5.0)))    # straddles the doorway wall
    except AssertionError:
        return
    raise AssertionError('expected a crowd area containing a wall to fail loudly')
