import numpy as np
import math

class MappingEngine:

    def __init__(self, grid, localization, max_range=150):
        self.grid = grid
        self.localization = localization
        self.max_range = max_range

        # Last ultrasonic reading (in cm)
        self.last_distance = None

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

        dist = float(self.last_distance)

        # Robot pose
        x, y, heading_deg = self.localization.get_pose()
        heading = math.radians(heading_deg)

        # Ray direction
        dx = math.cos(heading)
        dy = math.sin(heading)

        # March along the ray
        steps = int(dist)

        # FREE SPACE CARVING
        for i in range(1, steps):
            fx = int(x + i * dx)
            fy = int(y + i * dy)

            if not self.grid.in_bounds(fx, fy):
                break

            # Mark free space
            self.grid.mark_free(fx, fy)

        # Only mark obstacle if within range
        if dist < self.max_range:
            obs_x = int(x + dist * dx)
            obs_y = int(y + dist * dy)

            if self.grid.in_bounds(obs_x, obs_y):
                # Mark a 3x3 block to create a "thick" obstacle
                for ox in range(obs_x - 1, obs_x + 2):
                    for oy in range(obs_y - 1, obs_y + 2):
                        if self.grid.in_bounds(ox, oy):
                            self.grid.mark_occupied(ox, oy)
