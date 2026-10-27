"""Global planner: 8-connected A* on the inflated grid. Logic unchanged from nav_mppi.py."""

import heapq

import numpy as np


def astar(grid, start_xy, goal_xy, clearance=0.35):
    """-> (N, 2) world-frame path, or None if unreachable."""
    blocked = grid.dist <= clearance
    H, W = blocked.shape
    s = grid.to_cell(*start_xy)[::-1]   # (r, c)
    g = grid.to_cell(*goal_xy)[::-1]

    nbrs = [(dr, dc) for dr in (-1, 0, 1) for dc in (-1, 0, 1) if (dr, dc) != (0, 0)]
    h = lambda n: np.hypot(n[0] - g[0], n[1] - g[1])

    open_q = [(h(s), 0.0, s)]
    came, gscore = {}, {s: 0.0}
    while open_q:
        _, cost, cur = heapq.heappop(open_q)
        if cur == g:
            path = [cur]
            while path[-1] in came:
                path.append(came[path[-1]])
            path = path[::-1]
            return (np.array([[c, r] for r, c in path]) + 0.5) * grid.res + grid.origin
        if cost > gscore.get(cur, np.inf):
            continue
        for dr, dc in nbrs:
            nxt = (cur[0] + dr, cur[1] + dc)
            if not (0 <= nxt[0] < H and 0 <= nxt[1] < W) or blocked[nxt]:
                continue
            ng = cost + np.hypot(dr, dc)
            if ng < gscore.get(nxt, np.inf):
                gscore[nxt] = ng
                came[nxt] = cur
                heapq.heappush(open_q, (ng + h(nxt), ng, nxt))
    return None


def lookahead(path, xy, dist=1.5):
    """Pick the point on `path` roughly `dist` ahead of the closest point."""
    d = np.linalg.norm(path - xy, axis=1)
    i = int(np.argmin(d))
    seg = np.linalg.norm(np.diff(path[i:], axis=0), axis=1).cumsum()
    j = i + (1 + int(np.searchsorted(seg, dist))) if len(seg) else i
    return path[min(j, len(path) - 1)]
