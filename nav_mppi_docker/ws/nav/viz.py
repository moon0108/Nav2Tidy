"""Debug overlay: A* path, lookahead waypoint, MPPI samples and nominal plan.

Draws into any mjvScene -- `viewer.user_scn` for the interactive viewer, or a
`mujoco.Renderer`'s scene for a still (pass reset=False there, so the model's
own geoms survive).
"""

import mujoco
import numpy as np

GLOBAL_PATH = (0.2, 0.9, 0.3, 1.0)    # green
WAYPOINT = (1.0, 0.9, 0.2, 1.0)       # yellow
NOMINAL = (0.2, 0.8, 1.0, 1.0)        # cyan
SAMPLE = (1.0, 0.45, 0.1, 0.35)       # translucent orange

Z_PATH, Z_SAMPLE, Z_NOMINAL = 0.03, 0.06, 0.09


def _line(scn, p0, p1, rgba, width):
    if scn.ngeom >= scn.maxgeom:
        return False
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3),
                        np.eye(3).flatten(), np.asarray(rgba, np.float32))
    mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, width,
                         np.asarray(p0, float), np.asarray(p1, float))
    scn.ngeom += 1
    return True


def _polyline(scn, xy, z, rgba, width, stride=1):
    pts = np.asarray(xy)[::stride]
    for a, b in zip(pts[:-1], pts[1:]):
        if not _line(scn, (a[0], a[1], z), (b[0], b[1], z), rgba, width):
            return


def _sphere(scn, xy, z, radius, rgba):
    if scn.ngeom >= scn.maxgeom:
        return
    mujoco.mjv_initGeom(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
                        np.array([radius, 0, 0]), np.array([xy[0], xy[1], z]),
                        np.eye(3).flatten(), np.asarray(rgba, np.float32))
    scn.ngeom += 1


def draw(scn, path=None, waypoint=None, mppi=None, n_samples=24, reset=True):
    """path: (N, 2) A* path, waypoint: (2,), mppi: an MPPI that has stepped."""
    if reset:
        scn.ngeom = 0

    if path is not None and len(path) > 1:
        # A* returns one point per cell; every 4th keeps the geom count sane.
        _polyline(scn, path, Z_PATH, GLOBAL_PATH, 0.012, stride=4)
    if waypoint is not None:
        _sphere(scn, waypoint, Z_PATH, 0.08, WAYPOINT)

    if mppi is not None and getattr(mppi, 'last_X', None) is not None:
        # Nominal first: if the scene runs out of geoms, the plan survives and
        # only the sample cloud gets clipped.
        _polyline(scn, mppi.last_nominal[:, :2], Z_NOMINAL, NOMINAL, 0.02)
        X = mppi.last_X
        if n_samples:
            # An even slice of the cloud, not the top-weighted ones: with lam=0.5
            # the weights collapse onto a single sample (effective K ~ 1), so
            # "highest weight" would show one real rollout and n-1 arbitrary ones.
            idx = np.linspace(0, len(X) - 1, min(n_samples, len(X))).astype(int)
            stride = max(1, X.shape[1] // 8)
            for k in idx:
                _polyline(scn, X[k, :, :2], Z_SAMPLE, SAMPLE, 0.01, stride=stride)
