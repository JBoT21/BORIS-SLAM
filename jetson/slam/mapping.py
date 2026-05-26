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

        # For visualization
        self.last_distance = None

    
    # Called when a new ultrasonic reading arrives
    def update_from_ultrasonic(self, distance_cm):
        # Set to max range
        if distance_cm is None or distance_cm <= 0:
            self.last_distance = None
        else:
            self.last_distance = min(distance_cm, self.max_range)

    
    # Integrate the most recent reading into the occupancy grid
    def integrate(self):
        if self.last_distance is None:
            return
        
        dist = int(self.last_distance)
        # Robot pose
        x, y, heading_deg = self.localization.get_pose()
        heading = math.radians(heading_deg)

        # Ray direction
        dx = math.cos(heading)
        dy = math.sin(heading)

        # Free-space carving
        for i in range(1, dist):
            fx = int(x + i * dx)
            fy = int(y + i * dy)

            if not self.grid.in_bounds(fx, fy):
                break

            self.grid.mark_free(fx, fy)

        # Obstacle marking (only if within range)
        obs_x = int(x + dist * dx)
        obs_y = int(y + dist * dy)
        if self.grid.in_bounds(obs_x, obs_y):
            # Only mark obstacle if distance < max_range
            if dist < self.max_range:
                self.grid.mark_occupied(obs_x, obs_y)