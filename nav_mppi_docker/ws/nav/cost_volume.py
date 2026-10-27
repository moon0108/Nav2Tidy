"""Predicted pedestrian occupancy as a (n_steps, H, W) float32 volume in [0, 1].

This is the seam where a real detection/prediction module drops in later, so the
interface is fixed: ground-truth pedestrian states go in, a dense volume on the
*static grid's* origin and resolution comes out, index 0 meaning "now".

Latency is handled here rather than in MPPI._dyn_lookup: a volume built from a
snapshot of age `a` shifts its own time axis by `a`, so the consumer can always
read index t as "t control periods from now".
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class CostVolumeConfig:
    """exp(-(d - radius) / decay), clipped to [0, 1]: the same shape Grid.bake()
    uses for static obstacles, anchored on the contact distance.

    Two other shapes were measured and are worse (8 pedestrians, 4 seeds):
      - Gaussian on the pedestrian centre (the model PLAN section 6 describes):
        at the 0.65 m contact distance it is only ~0.1 of its peak, so it barely
        competes with the goal term -- collision rate 0.50 at best.
      - flat disk with a Gaussian taper: the plateau removes the gradient exactly
        where the robot needs it, and MPPI cannot descend a flat cost -- 1.00.
    """
    radius: float = 0.65         # m, robot radius + pedestrian radius: contact distance
    radius_growth: float = 0.05  # m per second of prediction
    decay0: float = 0.3          # m, decay length outside the contact distance
    decay_growth: float = 0.1    # m per second -- uncertainty widens the tail
    trunc: float = 4.0           # truncate the tail at this many decay lengths
    # Per-step confidence in the constant-velocity prediction -- the time discount
    # PLAN section 7 prescribes for the freezing/livelock failure mode. A
    # pedestrian pacing back and forth paints a 2.5 s corridor the robot treats as
    # a wall, and the two of them livelock. Discounting the far end of the
    # prediction buys arrivals at the price of clearance (4 pedestrians, 8 seeds):
    #   1.00 -> success 0.88, collision rate 0.12, 35 s   (default: collisions hurt
    #   0.95 -> success 0.88, collision rate 0.25, 28 s    more than timeouts)
    #   0.90 -> success 1.00, collision rate 0.38, 15 s
    #   0.85 -> success 1.00, collision rate 0.50, 17 s
    confidence_decay: float = 1.0
    latency: float = 0.0         # s of artificially injected pipeline delay
    compensate: bool = True      # shift the time axis by the volume's age


def build_cost_volume(pos, vel, grid, cfg: CostVolumeConfig = None, n_steps=25, dt=0.1, age=0.0):
    """pos, vel: (n, 2) world frame -> (n_steps, H, W) float32 in [0, 1].

    Constant-velocity extrapolation, one kernel per pedestrian per step.
    Pedestrians are combined with max(), not sum, so the volume stays in [0, 1].
    """
    cfg = cfg or CostVolumeConfig()
    H, W = grid.occ.shape
    cv = np.zeros((n_steps, H, W), np.float32)
    pos, vel = np.atleast_2d(pos), np.atleast_2d(vel)
    res = grid.res

    for t in range(n_steps):
        lead = t * dt + (age if cfg.compensate else 0.0)
        decay = cfg.decay0 + cfg.decay_growth * lead
        radius = cfg.radius + cfg.radius_growth * lead
        rad = int(np.ceil((radius + cfg.trunc * decay) / res))
        confidence = cfg.confidence_decay ** (lead / dt)   # staleness counts too
        centres = pos + vel * lead
        for cx, cy in centres:
            c, r = grid.to_cell(cx, cy)
            c0, c1 = max(c - rad, 0), min(c + rad + 1, W)
            r0, r1 = max(r - rad, 0), min(r + rad + 1, H)
            if c0 >= c1 or r0 >= r1:
                continue                                    # fully outside the grid
            # Distance from the true (sub-cell) centre, not the cell index.
            dx = (np.arange(c0, c1) + 0.5) * res + grid.origin[0] - cx
            dy = (np.arange(r0, r1) + 0.5) * res + grid.origin[1] - cy
            d = np.sqrt(dy[:, None] ** 2 + dx[None, :] ** 2)
            patch = np.minimum(np.exp(-(d - radius) / decay), 1.0) * confidence
            np.maximum(cv[t, r0:r1, c0:c1], patch, out=cv[t, r0:r1, c0:c1])
    return cv


class DelayedCrowdSensor:
    """Holds pedestrian states back by `latency` seconds so the planner sees a
    stale snapshot, the way a real detector would."""

    def __init__(self, cfg: CostVolumeConfig, dt=0.1):
        self.dt = dt
        self.n_delay = int(round(cfg.latency / dt))
        self.buffer = []

    def observe(self, pos, vel):
        """-> (pos, vel, age_seconds) of the snapshot the planner is allowed to use."""
        self.buffer.append((np.array(pos), np.array(vel)))
        if len(self.buffer) > self.n_delay + 1:
            self.buffer.pop(0)
        p, v = self.buffer[0]
        return p, v, self.dt * (len(self.buffer) - 1)
