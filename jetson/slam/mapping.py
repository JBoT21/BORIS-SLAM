import numpy as np
import math

class MappingEngine:

    def __init__(self, grid, localization, max_range=150):
        self.grid = grid
        self.localization = localization
        self.max_range = max_range

        # Last ultrasonic reading (cm)
        self.last_distance = None

        # Size of each grid cell in cm
        self.cell_size_cm = 30.0

    # Called when a new ultrasonic reading arrives
    def update_from_ultrasonic(self, distance_cm):
        if distance_cm is None or distance_cm <= 0:
            self.last_distance = None
            return

        # Clamp to max range
        self.last_distance = min(distance_cm, self.max_range)

    # Integrate the most recent reading into the occupancy grid
    def integrate(self):
        if self.last_distance is None:
            return

        # Convert cm → grid cells (each cell = 30 cm)
        dist_cells = self.last_distance / self.cell_size_cm
        steps = int(dist_cells)

        # Robot pose (already in grid units)
        x, y, heading_deg = self.localization.get_pose()
        heading = math.radians(heading_deg)

        # Ray direction
        dx = math.cos(heading)
        dy = math.sin(heading)

        #Free space carving
        for i in range(1, steps):
            fx = int(x + i * dx)
            fy = int(y + i * dy)

            if not self.grid.in_bounds(fx, fy):
                break

            self.grid.mark_free(fx, fy)

        #Obstacle marking
        obs_x = int(x + steps * dx)
        obs_y = int(y + steps * dy)

        if self.grid.in_bounds(obs_x, obs_y):

            # Mark a small block to make obstacles visible
            for ox in range(obs_x - 1, obs_x + 2):
                for oy in range(obs_y - 1, obs_y + 2):
                    if self.grid.in_bounds(ox, oy):
                        self.grid.mark_occupied(ox, oy)
