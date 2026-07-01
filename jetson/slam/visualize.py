"""
Responsibilities:

1. Display map with Matplotlib
2. Show robot pose
3. Show sensor rays
4. Save frames for debugging (headless-safe)
"""

import matplotlib
matplotlib.use("Agg")   # IMPORTANT: headless backend for Jetson/systemd

import matplotlib.pyplot as plt
import numpy as np
import os


class Visualizer:
    """
    Displays:
        - Occupancy grid
        - Robot pose
        - Heading arrow
        - Sensor rays (optional)
        - Saves frames (optional)
    """

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
