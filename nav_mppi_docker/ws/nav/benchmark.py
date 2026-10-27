"""Density x seed benchmark -> CSV.

Variants are (name, CostVolumeConfig or None), so the Phase 1 planner (no
dynamic cost at all) is just another row and stays comparable.
"""

import argparse
import csv
from dataclasses import replace
from pathlib import Path

import numpy as np

from nav import scene
from nav.cost_volume import CostVolumeConfig
from nav.grid import plan_grid, static_grid
from nav.mppi import MPPIConfig
from nav.pedestrians import Crowd, CrowdConfig
from nav.runner import NavSim, RunConfig, navigate

# name -> (MPPIConfig override or None, CostVolumeConfig or None)
VARIANTS = {
    'baseline': (None, None),                                      # phase 1 planner, blind
    'cost_volume': (None, CostVolumeConfig()),
    'decay_0.9': (None, CostVolumeConfig(confidence_decay=0.9)),   # anti-livelock knob
    'latency_0.15': (None, CostVolumeConfig(latency=0.15)),
    'latency_0.15_uncompensated': (None, CostVolumeConfig(latency=0.15, compensate=False)),
}


# Pedestrians per CROWD_AREA (28.5 m^2) -> 0.07 / 0.14 / 0.28 per m^2. The top
# density is already well above the 0.04-0.1 per m^2 typical of crowd navigation
# benchmarks, so it is the stress case rather than the nominal one.
DENSITIES = (2, 4, 8)


def run(densities=DENSITIES, seeds=20, variants=VARIANTS, policy='sfm',
        sees_robot=False, out='nav_benchmark.csv', verbose=True):
    grid, pgrid = static_grid(), plan_grid()
    rows = []
    for n_peds in densities:
        sim = NavSim(n_peds=n_peds, show_viewer=False, real_time=False)
        for name, (mppi_cfg, cv_cfg) in variants.items():
            for seed in range(seeds):
                crowd = Crowd(CrowdConfig(n=n_peds, policy=policy, sees_robot=sees_robot, seed=seed))
                cfg = RunConfig(start=scene.CROWD_START, goal=scene.CROWD_GOAL,
                                mppi=replace(mppi_cfg or MPPIConfig(), seed=seed))
                r = navigate(sim, cfg, grid=grid, pgrid=pgrid, crowd=crowd,
                             cv_cfg=cv_cfg, verbose=False)
                rows.append(dict(variant=name, n_peds=n_peds, seed=seed, policy=policy,
                                 sees_robot=int(sees_robot), **r))
            if verbose:
                print(f'  {n_peds:2d} peds  {name:26s} {_line(rows[-seeds:])}')
            _write(out, rows)   # after every variant: a full sweep takes hours
        sim.close()

    if verbose and out:
        print(f'wrote {len(rows)} episodes to {out}')
    return rows


def _write(path, rows):
    if not path or not rows:
        return
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def _line(rows):
    return (f"success={np.mean([r['success'] for r in rows]):.2f} "
            f"collision_rate={np.mean([r['ped_collisions'] > 0 for r in rows]):.2f} "
            f"collisions={sum(r['ped_collisions'] for r in rows):3d} "
            f"min_dist={min(r['min_ped_dist'] for r in rows):.2f} "
            f"t={np.mean([r['nav_time'] for r in rows]):5.1f}s "
            f"len={np.mean([r['path_length'] for r in rows]):5.2f}m "
            f"jerk={np.mean([r['mean_jerk'] for r in rows]):.2f} "
            f"freeze={np.mean([r['freezing_time'] for r in rows]):4.1f}s")


def summarise(rows):
    """Aggregate by (variant, n_peds) -- the table that goes in the report."""
    keys = sorted({(r['variant'], r['n_peds']) for r in rows}, key=lambda k: (k[1], k[0]))
    print(f"\n{'variant':28s} {'peds':>4s}  summary")
    for variant, n in keys:
        sel = [r for r in rows if r['variant'] == variant and r['n_peds'] == n]
        print(f'{variant:28s} {n:4d}  {_line(sel)}')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--densities', type=int, nargs='+', default=list(DENSITIES))
    p.add_argument('--seeds', type=int, default=20)
    p.add_argument('--variants', nargs='+', default=list(VARIANTS), choices=list(VARIANTS))
    p.add_argument('--policy', default='sfm', choices=['constant', 'waypoint', 'sfm'])
    p.add_argument('--ped-sees-robot', action='store_true')
    p.add_argument('--out', default='nav_benchmark.csv')
    args = p.parse_args()
    rows = run(densities=args.densities, seeds=args.seeds,
               variants={k: VARIANTS[k] for k in args.variants},
               policy=args.policy, sees_robot=args.ped_sees_robot, out=args.out)
    summarise(rows)


if __name__ == '__main__':
    main()
