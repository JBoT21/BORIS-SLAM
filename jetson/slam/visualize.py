import cv2
import numpy as np

COLOR_UNKNOWN = (100, 100, 100)
COLOR_FREE = (200, 200, 200)
COLOR_OCCUPIED = (0, 0, 0)
COLOR_ROBOT = (0, 0, 255)

ARROWS = {
    0: (0, -1),   # North
    1: (1, 0),    # East
    2: (0, 1),    # South
    3: (-1, 0),   # West
}

CELL_SIZE = 12

class Visualizer:
    def __init__(self, mapper):
        self.mapper = mapper

    def update(self, heading_index):
        grid = self.mapper.get_grid()
        h, w = grid.shape

        img = np.zeros((h * CELL_SIZE, w * CELL_SIZE, 3), dtype=np.uint8)

        # Draw grid
        for y in range(h):
            for x in range(w):
                val = grid[y][x]
                if val == -1:
                    color = COLOR_UNKNOWN
                elif val == 0:
                    color = COLOR_FREE
                else:
                    color = COLOR_OCCUPIED

                cv2.rectangle(
                    img,
                    (x * CELL_SIZE, y * CELL_SIZE),
                    ((x + 1) * CELL_SIZE, (y + 1) * CELL_SIZE),
                    color,
                    -1
                )

        # Draw robot
        rx, ry = self.mapper.rx, self.mapper.ry
        cv2.rectangle(
            img,
            (rx * CELL_SIZE, ry * CELL_SIZE),
            ((rx + 1) * CELL_SIZE, (ry + 1) * CELL_SIZE),
            COLOR_ROBOT,
            -1
        )

        # Draw heading arrow
        dx, dy = ARROWS[heading_index]
        cx = rx * CELL_SIZE + CELL_SIZE // 2
        cy = ry * CELL_SIZE + CELL_SIZE // 2
        ex = cx + dx * CELL_SIZE
        ey = cy + dy * CELL_SIZE

        cv2.arrowedLine(img, (cx, cy), (ex, ey), (0, 0, 255), 2)

        cv2.imshow("BORIS SLAM-Lite", img)
        cv2.waitKey(1)
