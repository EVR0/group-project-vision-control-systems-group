"""
Hybrid A* (SE(2)) path planner — 4-direction cardinal motion.

State: (col, row, yaw_bin)
Supports forward and backward motion.
"""

import heapq
import math
import logging
from typing import Optional, List, Dict, Tuple

from grid_map import GridMap

log = logging.getLogger(__name__)

# 4 cardinal directions — forward and backward
DIRECTIONS_FWD = [
    ( 0, -1, 1.0, 0, False),   # North
    ( 1,  0, 1.0, 2, False),   # East
    ( 0,  1, 1.0, 4, False),   # South
    (-1,  0, 1.0, 6, False),   # West
]
DIRECTIONS_REV = [
    ( 0,  1, 1.2, 0, True),    # Reverse-N
    (-1,  0, 1.2, 2, True),    # Reverse-E
    ( 0, -1, 1.2, 4, True),    # Reverse-S
    ( 1,  0, 1.2, 6, True),    # Reverse-W
]
DIRECTIONS = DIRECTIONS_FWD + DIRECTIONS_REV

NUM_YAW_BINS = 8
YAW_BIN_RAD = 2.0 * math.pi / NUM_YAW_BINS


def yaw_bin_to_rad(yaw_bin: int) -> float:
    rad = yaw_bin * YAW_BIN_RAD
    return _wrap(rad)


def rad_to_yaw_bin(rad: float) -> int:
    rad = _wrap(rad)
    if rad < 0:
        rad += 2.0 * math.pi
    return int(round(rad / YAW_BIN_RAD)) % NUM_YAW_BINS


def _wrap(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _yaw_distance(a: int, b: int) -> int:
    d = abs(a - b)
    return min(d, NUM_YAW_BINS - d)


class HybridAStar:
    def __init__(self, grid: GridMap, turn_penalty: float = 0.5,
                 pos_tol_m: float = 0.20, yaw_tol_rad: float = 0.35,
                 max_iterations: int = 500_000):
        self.grid = grid
        self.turn_penalty = turn_penalty
        self.pos_tol_m = pos_tol_m
        self.yaw_tol_rad = yaw_tol_rad
        self.max_iterations = max_iterations
        self._pos_tol_cells = max(0, int(pos_tol_m / grid.resolution))

    def search(self, start_x, start_y, start_yaw,
               goal_x, goal_y, goal_yaw) -> Optional[List[Dict[str, float]]]:
        grid = self.grid
        sc, sr = grid.world_to_grid(start_x, start_y)
        gc, gr = grid.world_to_grid(goal_x, goal_y)
        sb = rad_to_yaw_bin(start_yaw)
        gb = rad_to_yaw_bin(goal_yaw)

        if not grid.is_free(sc, sr):
            log.warning("Start inside obstacle!")
            return None
        if not grid.is_free(gc, gr):
            log.warning("Goal inside obstacle!")
            return None

        start_state = (sc, sr, sb)
        g_score = {start_state: 0.0}
        came_from = {}
        counter = 0
        open_set = []

        h0 = self._heuristic(sc, sr, sb, gc, gr, gb)
        heapq.heappush(open_set, (h0, counter, start_state))
        counter += 1
        iterations = 0

        while open_set and iterations < self.max_iterations:
            iterations += 1
            _, _, cur = heapq.heappop(open_set)
            cc, cr, cb = cur

            if self._is_goal(cc, cr, cb, gc, gr, gb):
                path = self._reconstruct(came_from, cur, grid)
                log.info(f"A* found path: {len(path)} waypoints, {iterations} iters")
                return path

            cur_g = g_score[cur]

            for dx, dy, step_cost, dir_bin, is_rev in DIRECTIONS:
                yaw_diff = _yaw_distance(cb, dir_bin)
                turn_cost = self.turn_penalty * yaw_diff
                rev_cost = 0.5 if is_rev else 0.0

                nc, nr, nb = cc + dx, cr + dy, dir_bin
                if not grid.is_free(nc, nr):
                    continue

                tent_g = cur_g + step_cost + turn_cost + rev_cost
                nstate = (nc, nr, nb)
                if tent_g < g_score.get(nstate, math.inf):
                    g_score[nstate] = tent_g
                    came_from[nstate] = cur
                    h = self._heuristic(nc, nr, nb, gc, gr, gb)
                    heapq.heappush(open_set, (tent_g + h, counter, nstate))
                    counter += 1

        log.warning(f"A* failed after {iterations} iterations")
        return None

    def _heuristic(self, c, r, b, gc, gr, gb):
        dx, dy = c - gc, r - gr
        return math.sqrt(dx * dx + dy * dy) + _yaw_distance(b, gb) * 0.1

    def _is_goal(self, c, r, b, gc, gr, gb):
        if abs(c - gc) > self._pos_tol_cells or abs(r - gr) > self._pos_tol_cells:
            return False
        yaw_diff = abs(_wrap(yaw_bin_to_rad(b) - yaw_bin_to_rad(gb)))
        return yaw_diff <= self.yaw_tol_rad

    @staticmethod
    def _reconstruct(came_from, end, grid):
        states = []
        cur = end
        while cur in came_from:
            states.append(cur)
            cur = came_from[cur]
        states.append(cur)
        states.reverse()

        path = []
        for c, r, b in states:
            wx, wy = grid.grid_to_world(c, r)
            path.append({"x": wx, "y": wy, "yaw": yaw_bin_to_rad(b)})
        return path


def snap_to_cardinal_bin(yaw_rad: float) -> int:
    cardinals = [0, 2, 4, 6]
    best, best_d = 0, float("inf")
    for b in cardinals:
        d = abs(_wrap(yaw_bin_to_rad(b) - yaw_rad))
        if d < best_d:
            best_d, best = d, b
    return best
