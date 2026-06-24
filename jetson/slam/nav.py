import math
import time

class Navigator:
    def __init__(self, grid, localization):
        self.grid = grid
        self.localization = localization

        self.forward_speed = 120
        self.turn_speed = 100

        # State machine
        self.state = "FORWARD"
        self.state_start = time.time()

        # Parameters
        self.turn_duration = 0.6       # commit to turn for 0.6s
        self.heading_tolerance = 12    # degrees
        self.obstacle_threshold = 1    # any obstacle in first cell

    def decide_next_move(self):
        x, y, heading = self.localization.get_pose()
        ahead = self._cells_ahead(x, y, heading)

        obstacle = self._is_obstacle(ahead)
        unknown = self._is_unknown(ahead)
        free = self._is_free(ahead)

        now = time.time()

        
        # STATE: FORWARD
        if self.state == "FORWARD":
            if obstacle:
                # pick a turn direction based on heading
                self.state = "TURN_LEFT" if (heading % 360) < 180 else "TURN_RIGHT"
                self.state_start = now
                return "L" if self.state == "TURN_LEFT" else "R"

            # explore unknown
            if unknown:
                return "F"

            # continue forward
            if free:
                return "F"

            # fallback
            self.state = "TURN_LEFT"
            self.state_start = now
            return "L"

        
        # STATE: TURN_LEFT
        if self.state == "TURN_LEFT":
            # commit to turn for minimum duration
            if now - self.state_start < self.turn_duration:
                return "L"

            # after turn duration, go forward again
            self.state = "FORWARD"
            return "F"

        
        # STATE: TURN_RIGHT
        if self.state == "TURN_RIGHT":
            if now - self.state_start < self.turn_duration:
                return "R"

            self.state = "FORWARD"
            return "F"

        # fallback
        return "F"

    # Helper functions
    def _cells_ahead(self, x, y, heading):
        cells = []
        rad = math.radians(heading)
        for d in range(1, 4):
            cx = int(x + d * math.cos(rad))
            cy = int(y + d * math.sin(rad))
            if self.grid.in_bounds(cx, cy):
                cells.append((cx, cy))
        return cells

    def _is_obstacle(self, cells):
        return any(self.grid.get(x, y) == 2 for x, y in cells)

    def _is_unknown(self, cells):
        return any(self.grid.get(x, y) == 0 for x, y in cells)

    def _is_free(self, cells):
        return all(self.grid.get(x, y) == 1 for x, y in cells)
