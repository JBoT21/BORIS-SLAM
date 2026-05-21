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

def integrate():
    # Placeholder for integrating new sensor data into the occupancy grid.
    # This function will take the latest sensor readings and update the grid accordingly.
    pass

def update_from_ultrasonic():
    # Placeholder for processing ultrasonic sensor data and updating the occupancy grid.
    pass



