import numpy as np

# Occupancy values
UNKNOWN = -1
FREE = 0
OCCUPIED = 2

# Grid size (adjust as needed)
GRID_W = 50
GRID_H = 50

# Cell size in cm
CELL_SIZE = 25

# Max ultrasonic range to consider (in cm)
MAX_RANGE = 400

# Heading index → direction vectors
DIRS = {
    0: (0, 1),    # North
    1: (1, 0),    # East
    2: (0, -1),   # South
    3: (-1, 0),   # West
}

class MappingEngine:
    def __init__(self):
        self.grid = np.full((GRID_H, GRID_W), UNKNOWN, dtype=np.int8)

        # Start robot in center of map
        self.rx = GRID_W // 2
        self.ry = GRID_H // 2

    def update_from_packet(self, packet):
        """
        Packet format:
            U:<dist_cm>,H:<heading_index>
        """

        try:
            parts = packet.strip().split(",")
            dist = float(parts[0].split(":")[1])
            heading_index = int(parts[1].split(":")[1])
        except Exception as e:
            print("[Mapper] Parse error:", e)
            return

        dx, dy = DIRS[heading_index]

        # Number of cells the ultrasonic ray reaches
        max_cells = min(int(MAX_RANGE / CELL_SIZE), 10)

        # Mark free cells along the ray
        for i in range(1, max_cells + 1):
            cx = self.rx + dx * i
            cy = self.ry + dy * i

            # Bounds check
            if cx < 0 or cx >= GRID_W or cy < 0 or cy >= GRID_H:
                break

            # Convert distance to cell index
            if i * CELL_SIZE < dist:
                self.grid[cy][cx] = FREE
            else:
                self.grid[cy][cx] = OCCUPIED
                break

    def get_grid(self):
        return self.grid





"""import math

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
"""