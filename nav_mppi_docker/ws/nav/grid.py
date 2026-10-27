"""Occupancy grid + baked soft cost field. Algorithm unchanged from nav_mppi.py."""

import numpy as np
from scipy.ndimage import distance_transform_edt

from nav import scene


class Grid:
    def __init__(self, origin, res, shape):
        self.origin = np.asarray(origin, float)  # world coord of cell (0,0)
        self.res = res
        self.occ = np.zeros(shape, bool)         # (H, W), indexed [row, col] = [y, x]
        self._cost = None

    def add_box(self, xmin, ymin, xmax, ymax):
        c0, r0 = self.to_cell(xmin, ymin)
        c1, r1 = self.to_cell(xmax, ymax)
        self.occ[max(r0, 0):r1 + 1, max(c0, 0):c1 + 1] = True
        self._cost = None

    def to_cell(self, x, y):
        c = int(np.floor((x - self.origin[0]) / self.res))
        r = int(np.floor((y - self.origin[1]) / self.res))
        return c, r

    def to_world(self, c, r):
        """Cell centre in world coords -- inverse of to_cell up to res/2."""
        return (np.asarray([c, r], float) + 0.5) * self.res + self.origin

    def bake(self, robot_radius=0.35, decay=0.15):
        """Signed-distance based soft cost field, computed once."""
        d = distance_transform_edt(~self.occ) * self.res
        cost = np.exp(-(d - robot_radius) / decay)
        cost[d <= robot_radius] = 10.0
        self._cost = np.clip(cost, 0.0, 10.0)
        self.dist = d
        return self

    def lookup(self, pts):
        """pts: (..., 2) world xy -> (...,) cost. Nearest-neighbour, clamped."""
        idx = np.floor((pts - self.origin) / self.res).astype(np.int32)
        H, W = self.occ.shape
        c = np.clip(idx[..., 0], 0, W - 1)
        r = np.clip(idx[..., 1], 0, H - 1)
        return self._cost[r, c]

    @classmethod
    def from_boxes(cls, boxes, origin=scene.GRID_ORIGIN, res=scene.GRID_RES, shape=scene.GRID_SHAPE):
        """boxes: list of (xmin, ymin, xmax, ymax) world metres."""
        g = cls(origin, res, shape)
        H, W = shape
        xmax_w, ymax_w = g.to_world(W - 1, H - 1)
        for b in boxes:
            # add_box slices with raw indices; a box poking outside the grid would
            # silently wrap and corrupt the map, so fail loudly instead.
            assert b[0] >= origin[0] and b[1] >= origin[1], f'box {b} starts outside grid'
            assert b[2] <= xmax_w and b[3] <= ymax_w, f'box {b} ends outside grid'
            g.add_box(*b)
        return g


def static_grid(res=scene.GRID_RES, robot_radius=scene.ROBOT_RADIUS, decay=0.15):
    """The scene's walls, rasterised and baked. Same source as the MJCF."""
    shape = (round(scene.GRID_SPAN / res),) * 2
    return Grid.from_boxes(scene.WALLS, res=res, shape=shape).bake(robot_radius=robot_radius, decay=decay)


def nearest_free(grid, xy, clearance):
    """Snap a world point to the closest cell with clearance to spare.

    A* expands no node whose cell is inflated, so a robot that has grazed a wall
    (or a goal placed too close to one) would otherwise make the search return
    None and strand the episode.
    """
    free = np.argwhere(grid.dist > clearance)          # (N, 2) as [row, col]
    if not len(free):
        return np.asarray(xy, float)
    w = (free[:, ::-1] + 0.5) * grid.res + grid.origin  # -> world xy
    return w[np.argmin(np.linalg.norm(w - np.asarray(xy, float), axis=1))]


def is_free(grid, xy, clearance):
    c, r = grid.to_cell(xy[0], xy[1])
    H, W = grid.occ.shape
    return 0 <= r < H and 0 <= c < W and grid.dist[r, c] > clearance


def plan_grid():
    """Coarser grid for A* only -- 8-connected search over 220x220 cells costs
    more than one control period."""
    return static_grid(res=scene.PLAN_RES)
