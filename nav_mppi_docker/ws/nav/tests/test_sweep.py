import numpy as np

from nav.cost_volume import CostVolumeConfig
from nav.mppi import MPPI, MPPIConfig
from nav.grid import Grid
from nav.sweep import build_variants


def test_int_fields_stay_ints():
    """T and K reach numpy as shape arguments; floats there raise TypeError."""
    variants = build_variants(['T=10,25', 'K=64'], [])
    for name, (mppi_cfg, _) in variants.items():
        assert isinstance(mppi_cfg.T, int), name
        assert isinstance(mppi_cfg.K, int), name
    grid = Grid(origin=(-1.0, -1.0), res=0.05, shape=(60, 60)).bake(robot_radius=0.35)
    cfg = variants['T=10_K=64'][0]
    assert np.all(np.isfinite(MPPI(grid, cfg).step(np.zeros(3), np.array([2.0, 1.0]))))


def test_bool_fields_parse():
    variants = build_variants([], ['compensate=False,True'])
    assert variants['compensate=False'][1].compensate is False
    assert variants['compensate=True'][1].compensate is True


def test_float_fields_and_cross_product():
    variants = build_variants(['w_dyn=30,60'], ['decay0=0.3'])
    assert len(variants) == 2
    assert variants['w_dyn=30.0_decay0=0.3'][0].w_dyn == 30.0
    assert variants['w_dyn=30.0_decay0=0.3'][1].decay0 == 0.3


def test_unknown_field_fails_loudly():
    try:
        build_variants(['not_a_field=1'], [])
    except AssertionError:
        return
    raise AssertionError('expected an unknown config field to fail loudly')
