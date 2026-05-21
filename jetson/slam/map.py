"""
This is the occupancy grid.

Responsibilities:

1. Create a 2D NumPy array (Done)

2. Mark free/occupied cells

3. Handle bounds checking

4. Save/load maps
"""
import numpy as np

def MappingEngine():
    # Create a 200x200 grid initialized to 0 (unknown)
    grid_size = 200
    occupancy_grid = np.zeros((grid_size, grid_size), dtype=int)
    return occupancy_grid

