"""
Responsibilities:

1. Sweep servo angles

2. Convert distance + angle → map coordinates

3. Update occupancy grid

4. Handle unknown space vs free space

(Most previously adapted code will go here)
"""

import numpy as np
import math


class MappingEngine:
  
    def __init__(self, grid, localization, max_range=150):
        self.grid = grid
        self.localization = localization
        self.max_range = max_range

    # Called when a new ultrasonic reading arrives
    def update_from_ultrasonic(self, distance_cm):       
        self.last_distance = distance_cm

    # Integrate the most recent reading into the occupancy grid
    def integrate(self):
        if not hasattr(self, "last_distance"):
            return  # no data yet

        dist = self.last_distance
        if dist is None:
            return

        # Robot pose
        x, y, heading_deg = self.localization.get_pose()

        # heading -> radians
        heading = math.radians(heading_deg)

        # Compute obstacle cell
        obs_x = int(x + dist * math.cos(heading))
        obs_y = int(y + dist * math.sin(heading))

        # Mark free space along the ray
        for i in range(1, dist):
            fx = int(x + i * math.cos(heading))
            fy = int(y + i * math.sin(heading))

            if self.grid.in_bounds(fx, fy):
                self.grid.mark_free(fx, fy)

        # Mark obstacle
        if self.grid.in_bounds(obs_x, obs_y):
            self.grid.mark_occupied(obs_x, obs_y)
