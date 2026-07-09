import pygame
import numpy as np

# Colors
COLOR_UNKNOWN = (100, 100, 100)
COLOR_FREE = (200, 200, 200)
COLOR_OCCUPIED = (0, 0, 0)
COLOR_ROBOT = (255, 0, 0)

# Heading arrows
ARROWS = {
    0: (0, -1),   # North
    1: (1, 0),    # East
    2: (0, 1),    # South
    3: (-1, 0),   # West
}

CELL_SIZE = 12  # pixels per cell

class Visualizer:
    def __init__(self, mapper):
        pygame.init()
        self.mapper = mapper

        self.width = mapper.grid.shape[1] * CELL_SIZE
        self.height = mapper.grid.shape[0] * CELL_SIZE

        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("BORIS SLAM-Lite Visualizer")

    def draw(self, heading_index):
        self.screen.fill((50, 50, 50))
        grid = self.mapper.get_grid()

        # Draw occupancy grid
        for y in range(grid.shape[0]):
            for x in range(grid.shape[1]):
                val = grid[y][x]
                if val == -1:
                    color = COLOR_UNKNOWN
                elif val == 0:
                    color = COLOR_FREE
                else:
                    color = COLOR_OCCUPIED

                pygame.draw.rect(
                    self.screen,
                    color,
                    (x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE)
                )

        # Draw robot
        rx, ry = self.mapper.rx, self.mapper.ry
        pygame.draw.rect(
            self.screen,
            COLOR_ROBOT,
            (rx * CELL_SIZE, ry * CELL_SIZE, CELL_SIZE, CELL_SIZE)
        )

        # Draw heading arrow
        dx, dy = ARROWS[heading_index]
        arrow_x = rx * CELL_SIZE + CELL_SIZE // 2
        arrow_y = ry * CELL_SIZE + CELL_SIZE // 2
        end_x = arrow_x + dx * CELL_SIZE
        end_y = arrow_y + dy * CELL_SIZE

        pygame.draw.line(
            self.screen,
            (255, 0, 0),
            (arrow_x, arrow_y),
            (end_x, end_y),
            3
        )

        pygame.display.flip()

    def update(self, heading_index):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()

        self.draw(heading_index)



"""
Responsibilities:

1. Display map with Matplotlib
2. Show robot pose
3. Show sensor rays
4. Save frames for debugging (headless-safe)


import matplotlib
matplotlib.use("Agg")   # IMPORTANT: headless backend for Jetson/systemd

import matplotlib.pyplot as plt
import numpy as np
import os


class Visualizer:
    Displays:
        - Occupancy grid
        - Robot pose
        - Heading arrow
        - Sensor rays (optional)
        - Saves frames (optional)


    def __init__(self, grid, localization, show_rays=False, save_frames=False):
        self.grid = grid
        self.localization = localization
        self.show_rays = show_rays
        self.save_frames = save_frames
        self.frame_id = 0

        # Create output directory if saving frames
        if self.save_frames:
            os.makedirs("imgs", exist_ok=True)

        # Create figure (non-interactive)
        self.fig, self.ax = plt.subplots(figsize=(6, 6))

        # Initial grid image
        self.im = self.ax.imshow(
            self.grid.grid.T,
            cmap="gray",
            origin="lower",
            vmin=0,
            vmax=2
        )
        self.fig.colorbar(self.im, ax=self.ax, fraction=0.046)

    def update(self):
        # Update grid
        self.im.set_data(self.grid.grid.T)

        # Clear previous markers
        self.ax.collections.clear()
        self.ax.patches.clear()

        # Robot pose
        x, y, heading = self.localization.get_pose()
        self.ax.plot(x, y, "ro", markersize=6)

        # Heading arrow
        dx = np.cos(np.radians(heading)) * 5
        dy = np.sin(np.radians(heading)) * 5
        self.ax.arrow(x, y, dx, dy, color="red", head_width=3)

        # Sensor ray
        if self.show_rays and hasattr(self.localization, "last_distance"):
            dist = self.localization.last_distance
            hx = x + dist * np.cos(np.radians(heading))
            hy = y + dist * np.sin(np.radians(heading))

            # Clip to map bounds
            hx = np.clip(hx, 0, self.grid.size - 1)
            hy = np.clip(hy, 0, self.grid.size - 1)

            self.ax.plot([x, hx], [y, hy], "y--", linewidth=1)

        # Axes settings
        self.ax.set_title("SLAM Visualization")
        self.ax.set_xlim(0, self.grid.size)
        self.ax.set_ylim(0, self.grid.size)
        self.ax.set_aspect("equal")

        # Save frame (headless-safe)
        if self.save_frames:
            filename = f"imgs/frame_{self.frame_id:05d}.png"
            self.fig.savefig(filename, dpi=120)
            self.frame_id += 1
"""