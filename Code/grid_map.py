"""
Grid Map — 2D occupancy grid for path planning.

Coordinate convention:
    x increases right  → grid column
    y increases down   → grid row
    Origin (0, 0) is top-left of the board
"""

import math
import logging
from typing import Optional, List, Tuple

log = logging.getLogger(__name__)


class GridMap:
    def __init__(self, resolution: float = 0.05,
                 board_width_m: float = 2.4384,
                 board_height_m: float = 2.4384,
                 grid_data: Optional[List[List[int]]] = None):
        self.resolution = resolution
        self.board_width_m = board_width_m
        self.board_height_m = board_height_m
        self.cols = int(math.ceil(board_width_m / resolution))
        self.rows = int(math.ceil(board_height_m / resolution))

        if grid_data is not None:
            self.grid = [row[:] for row in grid_data]
            self.rows = len(self.grid)
            self.cols = len(self.grid[0]) if self.rows > 0 else 0
        else:
            self.grid = [[0] * self.cols for _ in range(self.rows)]

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        return int(x / self.resolution), int(y / self.resolution)

    def grid_to_world(self, col: int, row: int) -> Tuple[float, float]:
        return (col + 0.5) * self.resolution, (row + 0.5) * self.resolution

    def in_bounds(self, col: int, row: int) -> bool:
        return 0 <= col < self.cols and 0 <= row < self.rows

    def is_free(self, col: int, row: int) -> bool:
        if not self.in_bounds(col, row):
            return False
        return self.grid[row][col] == 0

    def is_occupied(self, col: int, row: int) -> bool:
        return not self.is_free(col, row)

    def inflate(self, robot_radius_m: float) -> "GridMap":
        r_cells = int(math.ceil(robot_radius_m / self.resolution))
        if r_cells <= 0:
            return self
        inflated = GridMap(self.resolution, self.board_width_m, self.board_height_m)
        inflated.grid = [[0] * self.cols for _ in range(self.rows)]
        for r in range(self.rows):
            for c in range(self.cols):
                if self.grid[r][c] == 1:
                    for dr in range(-r_cells, r_cells + 1):
                        for dc in range(-r_cells, r_cells + 1):
                            nr, nc = r + dr, c + dc
                            if 0 <= nr < self.rows and 0 <= nc < self.cols:
                                if dr * dr + dc * dc <= r_cells * r_cells:
                                    inflated.grid[nr][nc] = 1
        return inflated

    def set_occupied(self, col: int, row: int) -> None:
        if self.in_bounds(col, row):
            self.grid[row][col] = 1

    def set_free(self, col: int, row: int) -> None:
        if self.in_bounds(col, row):
            self.grid[row][col] = 0
