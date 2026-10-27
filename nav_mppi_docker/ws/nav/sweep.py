"""Parameter sweep over MPPI / cost-volume knobs (PLAN section 7).

Each combination becomes a benchmark variant, so the output is the same CSV and
the same summary table as nav.benchmark.

    python -m nav.sweep --mppi w_dyn=60,150,300 --cv sigma0=0.5,0.7 --seeds 5
"""

import argparse
from dataclasses import fields, replace
from itertools import product

from nav.cost_volume import CostVolumeConfig
from nav.mppi import MPPIConfig

# nav.benchmark 는 main() 안에서 임포트한다. 그것이 nav.runner -> mujoco 를 끌어오는데,
# build_variants 는 설정만 다루므로 MuJoCo 가 필요 없다. 모듈 수준에 두면 ROS 이미지
# (MuJoCo 없음)에서 test_sweep.py 수집이 터지고, pytest 가 거기서 멈춰 **다른 테스트도
# 하나도 돌지 않는다**.


def _cast_for(t):
    """Field annotations arrive as real type objects, not strings -- comparing them
    to 'int' silently made every field a float, and int fields like T or K then
    crashed numpy on shape arguments."""
    if t in (bool, 'bool'):
        return lambda v: v.strip().lower() in ('1', 'true', 'yes')
    if t in (int, 'int'):
        return int
    return float


def _parse(spec, cls):
    """'w_dyn=60,150 gamma=0.98' -> {'w_dyn': [60.0, 150.0], 'gamma': [0.98]}"""
    types = {f.name: f.type for f in fields(cls)}
    out = {}
    for item in spec:
        key, _, values = item.partition('=')
        assert key in types, f'{cls.__name__} has no field {key}'
        cast = _cast_for(types[key])
        out[key] = [cast(v) for v in values.split(',')]
    return out


def build_variants(mppi_spec, cv_spec, base_mppi=None, base_cv=None):
    mppi_grid, cv_grid = _parse(mppi_spec, MPPIConfig), _parse(cv_spec, CostVolumeConfig)
    keys = list(mppi_grid) + list(cv_grid)
    variants = {}
    for combo in product(*(list(mppi_grid.values()) + list(cv_grid.values()))):
        assign = dict(zip(keys, combo))
        m = replace(base_mppi or MPPIConfig(), **{k: v for k, v in assign.items() if k in mppi_grid})
        c = replace(base_cv or CostVolumeConfig(), **{k: v for k, v in assign.items() if k in cv_grid})
        variants['_'.join(f'{k}={v}' for k, v in assign.items()) or 'default'] = (m, c)
    return variants


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--mppi', nargs='*', default=[], metavar='FIELD=V1,V2', help='e.g. w_dyn=60,150')
    p.add_argument('--cv', nargs='*', default=[], metavar='FIELD=V1,V2', help='e.g. sigma0=0.5,0.7')
    p.add_argument('--densities', type=int, nargs='+', default=[8])
    p.add_argument('--seeds', type=int, default=5)
    p.add_argument('--policy', default='sfm', choices=['constant', 'waypoint', 'sfm'])
    p.add_argument('--ped-sees-robot', action='store_true')
    p.add_argument('--out', default='nav_sweep.csv')
    args = p.parse_args()

    from nav.benchmark import run, summarise      # MuJoCo 가 필요한 지점은 여기부터다

    variants = build_variants(args.mppi, args.cv)
    print(f'{len(variants)} variants x {len(args.densities)} densities x {args.seeds} seeds')
    rows = run(densities=args.densities, seeds=args.seeds, variants=variants,
               policy=args.policy, sees_robot=args.ped_sees_robot, out=args.out)
    summarise(rows)


if __name__ == '__main__':
    main()
