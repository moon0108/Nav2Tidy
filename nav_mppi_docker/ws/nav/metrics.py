"""Episode metrics. Collisions are judged geometrically, never by MuJoCo contacts."""

import numpy as np

from nav import scene


class Metrics:
    def __init__(self, grid, dt=0.1, goal_tol=0.15, robot_radius=scene.ROBOT_RADIUS,
                 ped_radius=scene.PED_RADIUS, freeze_speed=0.05):
        self.grid, self.dt, self.goal_tol = grid, dt, goal_tol
        self.robot_radius, self.ped_radius, self.freeze_speed = robot_radius, ped_radius, freeze_speed
        self.success = False
        self.poses, self.controls, self.solve_ms, self.jitter_ms, self.cv_ms = [], [], [], [], []
        self.wall_hits, self.ped_hits = 0, 0
        self.min_wall_clear, self.min_ped_dist = np.inf, np.inf
        self._in_wall, self._in_ped = False, False

    def update(self, x, u, goal, solve_ms=0.0, jitter_ms=0.0, peds=None, cv_ms=0.0):
        """x: (3,) pose, u: (3,) command, peds: (n, 2) or None."""
        self.poses.append(np.asarray(x, float).copy())
        self.controls.append(np.asarray(u, float).copy())
        self.solve_ms.append(solve_ms)
        self.jitter_ms.append(jitter_ms)
        self.cv_ms.append(cv_ms)

        clear = float(self.grid.dist[self.grid.to_cell(x[0], x[1])[::-1]])
        self.min_wall_clear = min(self.min_wall_clear, clear)
        hit = clear < self.robot_radius   # strict: A* plans at exactly this clearance
        self.wall_hits += hit and not self._in_wall     # count entries, not steps
        self._in_wall = hit

        if peds is not None and len(peds):
            d = float(np.linalg.norm(np.asarray(peds) - x[:2], axis=1).min())
            self.min_ped_dist = min(self.min_ped_dist, d)
            hit_p = d < self.robot_radius + self.ped_radius
            self.ped_hits += hit_p and not self._in_ped
            self._in_ped = hit_p

    def summary(self, real_time=True):
        P = np.array(self.poses).reshape(-1, 3)
        steps = len(P)
        seg = np.linalg.norm(np.diff(P[:, :2], axis=0), axis=1) if steps > 1 else np.zeros(0)
        speed = seg / self.dt

        # |jerk| from the second difference of the executed velocity
        jerk = np.zeros(0)
        if steps > 3:
            vel = np.diff(P[:, :2], axis=0) / self.dt
            jerk = np.linalg.norm(np.diff(vel, n=2, axis=0), axis=1) / self.dt ** 2

        out = {
            'success': bool(self.success),
            'nav_time': steps * self.dt,
            'path_length': float(seg.sum()),
            'wall_collisions': int(self.wall_hits),
            'ped_collisions': int(self.ped_hits),
            'min_wall_clearance': float(self.min_wall_clear),
            'min_ped_dist': float(self.min_ped_dist),
            'mean_jerk': float(jerk.mean()) if len(jerk) else 0.0,
            'freezing_time': float(self.dt * (speed < self.freeze_speed).sum()),
            'solve_ms_mean': float(np.mean(self.solve_ms)) if steps else 0.0,
            'solve_ms_max': float(np.max(self.solve_ms)) if steps else 0.0,
            'cv_ms_mean': float(np.mean(self.cv_ms)) if steps else 0.0,
            'mean_speed': float(speed.mean()) if len(speed) else 0.0,
            'steps': steps,
        }
        if real_time and steps > 1:
            j = np.abs(self.jitter_ms[1:])
            out['jitter_ms_mean'] = float(j.mean())
            out['jitter_ms_max'] = float(j.max())
        return out
