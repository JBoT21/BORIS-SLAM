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

import math
import random

class Navigator:
    def __init__(self, grid, localization):
        self.grid = grid
        self.localization = localization
        self.forward_speed = 120
        self.turn_speed = 100
        
        # Exploration memory
        self.turn_direction = "LEFT"   # alternate to avoid loops
        self.visited = set()           # avoid revisiting same cells

   
    def decide_next_move(self):
        x, y, heading = self.localization.get_pose()

        # Mark current cell as visited
        self.visited.add((int(x), int(y)))

        ahead_cells = self._cells_ahead(x, y, heading)

        # 1. Obstacle ahead -> turn
        if self._is_obstacle(ahead_cells):
            return self._turn_command()

        # 2. Unknown space ahead -> explore
        if self._is_unknown(ahead_cells):
            return "F"   # forward

        # 3. Free space ahead -> continue
        if self._is_free(ahead_cells):
            return "F"

        # 4. Fallback: turn to escape dead‑end
        return self._turn_command()

 
    def _cells_ahead(self, x, y, heading):
        """Return grid cells 1–3 steps ahead."""
        cells = []
        rad = math.radians(heading)

        for d in range(1, 4):
            cx = int(x + d * math.cos(rad))
            cy = int(y + d * math.sin(rad))
            if self.grid.in_bounds(cx, cy):
                cells.append((cx, cy))

        return cells

    # CELL TYPE CHECKS
    def _is_obstacle(self, cells):
        return any(self.grid.get(x, y) == 2 for x, y in cells)

    def _is_unknown(self, cells):
        return any(self.grid.get(x, y) == 0 for x, y in cells)

    def _is_free(self, cells):
        return all(self.grid.get(x, y) == 1 for x, y in cells)

    def _turn_command(self):
        if self.turn_direction == "LEFT":
            self.turn_direction = "RIGHT"
            return "L"
        else:
            self.turn_direction = "LEFT"
            return "R"