"""
Responsiblities:
1.High-level decision making:
2.Move forward until obstacle
3. Turn left/right based on map
4.Explore unknown areas
5.Avoid revisiting mapped areas

Later, I will add:
A*,DWA,Q-learning or Frontier exploration (oooooh)

"""
"""
navigation.py — High-level decision making for Jetson Nano SLAM robot.
"""

import random
import math

class Navigator:
    def __init__(self, grid, localization):
        self.grid = grid
        self.localization = localization

        # Movement parameters
        self.forward_speed = 120
        self.turn_speed = 100

        # Exploration memory
        self.turn_direction = "LEFT"  # default bias

    def decide_next_move(self):
        x, y, heading = self.localization.get_pose()
        # Look ahead 1–3 cells
        ahead_cells = self._cells_ahead(x, y, heading)
        # If obstacle ahead -> turn
        if self._is_obstacle(ahead_cells):
            return self._turn_decision()
        # If unknown space ahead -> explore it
        if self._is_unknown(ahead_cells):
            return f"MOVE FWD {self.forward_speed}"
        # 3. If free space ahead -> continue
        if self._is_free(ahead_cells):
            return f"MOVE FWD {self.forward_speed}"
        # Random fallback
        return self._turn_decision()


    def _cells_ahead(self, x, y, heading):
        #Returns the grid cells 1 to 3 steps ahead of the robot.
        cells = []
        rad = math.radians(heading)

        for d in range(1, 4):
            cx = int(x + d * math.cos(rad))
            cy = int(y + d * math.sin(rad))
            if self.grid.in_bounds(cx, cy):
                cells.append((cx, cy))

        return cells

    #Helper functions
    def _is_obstacle(self, cells):
        return any(self.grid.get(x, y) == 2 for x, y in cells)

    def _is_unknown(self, cells):
        return any(self.grid.get(x, y) == 0 for x, y in cells)

    def _is_free(self, cells):
        return all(self.grid.get(x, y) == 1 for x, y in cells)

    
    # Turning logic
    def _turn_decision(self):
        #Later you can replace this with frontier exploration or A*.
        # Alternate turn direction to avoid loops
        if self.turn_direction == "LEFT":
            self.turn_direction = "RIGHT"
            return f"TURN LEFT {self.turn_speed}"
        else:
            self.turn_direction = "LEFT"
            return f"TURN RIGHT {self.turn_speed}"
