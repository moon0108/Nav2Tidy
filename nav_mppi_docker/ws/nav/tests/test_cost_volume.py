import numpy as np

from nav.cost_volume import CostVolumeConfig, DelayedCrowdSensor, build_cost_volume
from nav.grid import static_grid


def _peak(cv, grid, t):
    """Centroid of the plateau -- argmax is ambiguous once the kernel has a flat top."""
    r, c = np.nonzero(cv[t] >= cv[t].max() - 1e-6)
    return grid.to_world(c.mean(), r.mean())


def test_shape_dtype_and_range():
    grid = static_grid()
    cv = build_cost_volume([[5.0, 2.0]], [[0.0, 0.0]], grid, n_steps=25)
    assert cv.shape == (25,) + grid.occ.shape and cv.dtype == np.float32
    assert cv.min() >= 0.0 and cv.max() <= 1.0
    assert np.isclose(cv[0].max(), 1.0, atol=5e-3)   # peak sampled at a cell centre


def test_peak_sits_on_the_pedestrian():
    grid = static_grid()
    cv = build_cost_volume([[5.0, 2.0]], [[0.0, 0.0]], grid, n_steps=5)
    assert np.allclose(_peak(cv, grid, 0), [5.0, 2.0], atol=grid.res)


def test_constant_velocity_extrapolation():
    grid = static_grid()
    cv = build_cost_volume([[5.0, 2.0]], [[0.6, 0.0]], grid, n_steps=25, dt=0.1)
    assert np.allclose(_peak(cv, grid, 24), [5.0 + 0.6 * 2.4, 2.0], atol=2 * grid.res)


def test_uncertainty_grows_with_time():
    """The kernel widens with the prediction lead. Compare shapes, not absolute
    values -- confidence_decay scales later slices down."""
    grid = static_grid()
    cv = build_cost_volume([[5.0, 2.0]], [[0.0, 0.0]], grid, n_steps=25)
    c, r = grid.to_cell(5.8, 2.0)                  # 0.8 m to the side of a static pedestrian
    assert cv[24, r, c] / cv[24].max() > cv[0, r, c] / cv[0].max()


def test_confidence_decays_with_the_prediction_lead():
    grid = static_grid()
    args = ([[5.0, 2.0]], [[0.0, 0.0]], grid)
    faded = build_cost_volume(*args, CostVolumeConfig(confidence_decay=0.9), n_steps=25)
    assert faded[24].max() < 0.5 * faded[0].max()
    flat = build_cost_volume(*args, CostVolumeConfig(confidence_decay=1.0), n_steps=25)
    assert np.isclose(flat[24].max(), flat[0].max(), atol=1e-3)


def test_latency_shifts_the_time_axis():
    """A stale snapshot must be extrapolated forward, not read as if it were now."""
    grid = static_grid()
    args = ([[5.0, 2.0]], [[0.6, 0.0]], grid)
    fresh = build_cost_volume(*args, CostVolumeConfig(), n_steps=5, age=0.0)
    stale = build_cost_volume(*args, CostVolumeConfig(), n_steps=5, age=0.3)
    naive = build_cost_volume(*args, CostVolumeConfig(compensate=False), n_steps=5, age=0.3)
    assert np.allclose(_peak(stale, grid, 0), [5.0 + 0.6 * 0.3, 2.0], atol=2 * grid.res)
    assert np.allclose(_peak(naive, grid, 0), _peak(fresh, grid, 0), atol=grid.res)


def test_delayed_sensor_hands_back_a_stale_snapshot():
    s = DelayedCrowdSensor(CostVolumeConfig(latency=0.3), dt=0.1)
    for i in range(10):
        p, v, age = s.observe([[float(i), 0.0]], [[1.0, 0.0]])
        assert np.isclose(age, min(i, 3) * 0.1)
        assert p[0][0] == max(0.0, i - 3)


def test_zero_latency_is_a_pass_through():
    s = DelayedCrowdSensor(CostVolumeConfig(latency=0.0), dt=0.1)
    for i in range(5):
        p, _, age = s.observe([[float(i), 0.0]], [[0.0, 0.0]])
        assert age == 0.0 and p[0][0] == float(i)


def test_shares_the_static_grid_frame():
    """Origin/res mismatch between the volume and the grid silently mislocates
    every pedestrian, so the shapes must line up with the grid MPPI uses."""
    grid = static_grid()
    cv = build_cost_volume([[5.0, 2.0]], [[0.0, 0.0]], grid, n_steps=3)
    c, r = grid.to_cell(5.0, 2.0)
    assert cv[0, r, c] > 0.99
    assert cv.shape[1:] == grid.occ.shape
