"""
Pathfinder — for an open board with no obstacles, just returns
a direct path: [start, goal]. For boards with obstacles, uses A*.
"""

import math
import logging
from typing import Optional, List, Dict, Any

from grid_map import GridMap
from hybrid_astar import HybridAStar
import config

log = logging.getLogger(__name__)


def _simplify_path(path: List[Dict[str, float]]) -> List[Dict[str, float]]:
    """Keep only waypoints where direction changes, plus start and goal."""
    if len(path) <= 2:
        return path

    simplified = [path[0]]
    for i in range(1, len(path) - 1):
        prev = path[i - 1]
        curr = path[i]
        nxt = path[i + 1]
        dx1 = curr["x"] - prev["x"]
        dy1 = curr["y"] - prev["y"]
        dx2 = nxt["x"] - curr["x"]
        dy2 = nxt["y"] - curr["y"]
        if abs(dx1 - dx2) > 1e-6 or abs(dy1 - dy2) > 1e-6:
            simplified.append(curr)
    simplified.append(path[-1])
    return simplified


def plan_path(
    start: Dict[str, float],
    goal: Dict[str, float],
    params: Optional[Dict[str, Any]] = None,
    grid_data: Optional[List[List[int]]] = None,
) -> Optional[List[Dict[str, float]]]:
    """
    Plan a path from start to goal in board-frame metres / radians.
    Returns list of {"x", "y", "yaw"} waypoints, or None.
    """
    # For an open board (no obstacles), just go direct
    if grid_data is None and config.ROBOT_RADIUS_M == 0.0:
        log.info("Open board — using direct path (no A*)")
        return [
            {"x": start["x"], "y": start["y"], "yaw": start["yaw"]},
            {"x": goal["x"],  "y": goal["y"],  "yaw": goal["yaw"]},
        ]

    # Otherwise use A* with simplification
    defaults = {
        "resolution": config.GRID_RESOLUTION_M,
        "robot_radius_m": config.ROBOT_RADIUS_M,
        "turn_penalty": config.TURN_PENALTY,
        "pos_tol_m": 0.0,
        "yaw_tol_rad": config.YAW_TOLERANCE_RAD,
        "max_iterations": config.MAX_PLANNER_ITERATIONS,
        "board_width_m": config.BOARD_WIDTH_M,
        "board_height_m": config.BOARD_HEIGHT_M,
    }
    p = {**defaults, **(params or {})}

    grid = GridMap(
        resolution=p["resolution"],
        board_width_m=p["board_width_m"],
        board_height_m=p["board_height_m"],
        grid_data=grid_data,
    )
    grid = grid.inflate(p["robot_radius_m"])

    planner = HybridAStar(
        grid=grid,
        turn_penalty=p["turn_penalty"],
        pos_tol_m=p["pos_tol_m"],
        yaw_tol_rad=p["yaw_tol_rad"],
        max_iterations=p["max_iterations"],
    )

    path = planner.search(
        start["x"], start["y"], start["yaw"],
        goal["x"],  goal["y"],  goal["yaw"],
    )

    if path is None:
        log.warning("plan_path: NO PATH FOUND")
        return None

    path = _simplify_path(path)
    log.info(f"plan_path: {len(path)} waypoints (simplified)")
    return path
