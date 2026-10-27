import numpy as np

from nav.grid import static_grid
from nav.metrics import Metrics


def _straight(m, n=20, speed=0.5, dt=0.1, y=1.0, peds=None):
    goal = np.array([9.0, y])
    for k in range(n):
        x = np.array([1.0 + speed * dt * k, y, 0.0])
        m.update(x, np.array([speed, 0.0, 0.0]), goal, peds=peds)
    return m


def test_path_length_and_nav_time():
    s = _straight(Metrics(static_grid())).summary()
    assert np.isclose(s['path_length'], 19 * 0.05)
    assert np.isclose(s['nav_time'], 2.0)
    assert np.isclose(s['mean_speed'], 0.5)
    assert s['mean_jerk'] < 1e-9              # constant velocity has no jerk


def test_freezing_time_counts_slow_steps():
    s = _straight(Metrics(static_grid()), speed=0.0).summary()
    assert np.isclose(s['freezing_time'], 1.9)


def test_wall_collision_counts_entries_not_steps():
    m = Metrics(static_grid())
    goal = np.array([9.0, 1.0])
    for x in [1.0, 1.0, 3.15, 3.15, 3.15, 1.0, 3.15]:   # in and out of the doorway wall
        m.update(np.array([x, 1.0, 0.0]), np.zeros(3), goal)
    s = m.summary()
    assert s['wall_collisions'] == 2                    # two entries, not five steps
    assert s['min_wall_clearance'] == 0.0


def test_pedestrian_distance_and_collisions():
    peds = np.array([[2.0, 1.0]])
    s = _straight(Metrics(static_grid()), n=30, peds=peds).summary()
    assert s['ped_collisions'] == 1
    assert s['min_ped_dist'] < 0.65


def test_jerk_is_nonzero_when_the_command_oscillates():
    m, goal = Metrics(static_grid()), np.array([9.0, 1.0])
    for k in range(20):
        x = np.array([1.0 + 0.05 * k, 1.0 + 0.1 * (k % 2), 0.0])
        m.update(x, np.zeros(3), goal)
    assert m.summary()['mean_jerk'] > 1.0
