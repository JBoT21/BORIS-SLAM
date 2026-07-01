import math

class MappingEngine:
    def __init__(self, grid, localization, max_range=150):
        self.grid = grid
        self.localization = localization
        self.max_range = max_range

        # Last ultrasonic reading (cm)
        self.last_distance = None

        # Each grid cell = 30 cm
        self.cell_size_cm = 30.0

    def update_from_ultrasonic(self, distance_cm):
        print(f"[MappingEngine] Ultrasonic reading: {distance_cm} cm")
        if distance_cm is None or distance_cm <= 0:
            self.last_distance = None
            return

        # Clamp to max range
        self.last_distance = min(distance_cm, self.max_range)

    def integrate(self):
        if self.last_distance is None:
            return

        # Convert cm BACK grid cells
        dist_cells = self.last_distance / self.cell_size_cm

        # Prevent steps = 0 when distance < 30 cm
        steps = max(1, int(dist_cells))

        # Robot pose
        x, y, heading_deg = self.localization.get_pose()
        heading = math.radians(heading_deg)

        # Ray direction
        dx = math.cos(heading)
        dy = math.sin(heading)

        #Free space marking
        for i in range(0, steps):
            fx = int(x + i * dx)
            fy = int(y + i * dy)

            if not self.grid.in_bounds(fx, fy):
                break

            self.grid.mark_free(fx, fy)

        #Obstacle marking
        obs_x = int(x + steps * dx)
        obs_y = int(y + steps * dy)

        if self.grid.in_bounds(obs_x, obs_y):
            # Mark a small 3×3 block to make obstacles visible
            for ox in range(obs_x - 1, obs_x + 2):
                for oy in range(obs_y - 1, obs_y + 2):
                    if self.grid.in_bounds(ox, oy):
                        self.grid.mark_occupied(ox, oy)
