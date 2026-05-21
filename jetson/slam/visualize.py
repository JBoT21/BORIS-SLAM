"""
Responsibilities:

1. Display map with Matplotlib

2. Show robot pose

3. Show sensor rays?

4. Save frames for debugging?
"""
import matplotlib.pyplot as plt
import numpy as np


class Visualizer:
    """
    Displays:
        - Occupancy grid
        - Robot pose
        - Optional sensor rays
    """

    def __init__(self, grid, localization, show_rays=False):
        self.grid = grid
        self.localization = localization
        self.show_rays = show_rays

        # Matplotlib interactive mode
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(6, 6))

    def update(self):
        self.ax.clear()

        # Draw occupancy grid
        self.ax.imshow(
            self.grid.grid,
            cmap="gray",
            origin="lower",
            vmin=0,
            vmax=2
        )

        # Draw robot pose
        x, y, heading = self.localization.get_pose()
        self.ax.plot(x, y, "ro", markersize=6)

        # Draw heading arrow
        dx = np.cos(np.radians(heading)) * 5
        dy = np.sin(np.radians(heading)) * 5
        self.ax.arrow(x, y, dx, dy, color="red", head_width=3)

        # Draw sensor rays
        if self.show_rays and hasattr(self.localization, "last_distance"):
            dist = self.localization.last_distance
            hx = x + dist * np.cos(np.radians(heading))
            hy = y + dist * np.sin(np.radians(heading))
            self.ax.plot([x, hx], [y, hy], "y--", linewidth=1)

        self.ax.set_title("SLAM Visualization")
        self.ax.set_xlim(0, self.grid.size)
        self.ax.set_ylim(0, self.grid.size)
        self.ax.set_aspect("equal")

        plt.pause(0.001)
