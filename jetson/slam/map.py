"""
This is the occupancy grid.

Responsibilities:

1. Create a 2D NumPy array (Done)

2. Mark free/occupied cells

3. Handle bounds checking

4. Save/load maps
"""
"""
map.py — Occupancy grid for Jetson Nano SLAM system.
"""

import numpy as np
import os


class OccupancyGrid:
    """
    2D occupancy grid:
        0 = unknown
        1 = free
        2 = occupied
    """

    def __init__(self, size=200):
        self.size = size
        self.grid = np.zeros((size, size), dtype=np.uint8)

    def in_bounds(self, x, y):
        return 0 <= x < self.size and 0 <= y < self.size

    #Mark cells
    def mark_free(self, x, y):
        if self.in_bounds(x, y):
            self.grid[x, y] = 1

    def mark_occupied(self, x, y):
        if self.in_bounds(x, y):
            self.grid[x, y] = 2


    # Query
    def get(self, x, y):
        if self.in_bounds(x, y):
            return self.grid[x, y]
        return None

    # Save / Load
    def save(self, filename="map.npy"):
        np.save(filename, self.grid)
        print(f"[OccupancyGrid] Saved map to {filename}")

    def load(self, filename="map.npy"):
        if os.path.exists(filename):
            self.grid = np.load(filename)
            print(f"[OccupancyGrid] Loaded map from {filename}")
        else:
            print(f"[OccupancyGrid] No map found at {filename}")




