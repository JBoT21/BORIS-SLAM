from serial_link import read_ultrasonic, send
import numpy as np
import time

MAP_SIZE = 200
grid = np.zeros((MAP_SIZE, MAP_SIZE))

while True:
    distance = read_ultrasonic()
    if distance is not None:
        print(f"Distance: {distance} cm")
        # Update SLAM grid based on distance
        #Assume that the robot is at the "center" of the grid and facing upwards
        grid.fill(0)
        for i in range(MAP_SIZE):
            for j in range(MAP_SIZE):
                # Calculate the distance from the center to this cell
                cell_distance = np.sqrt((i - MAP_SIZE//2)**2 + (j - MAP_SIZE//2)**2)
                if cell_distance < distance:
                    grid[i, j] = 1

        #VERY BASIC SLAM: Just mark cells within the distance as occupied



    """send("MOVE FWD 150")
    time.sleep(0.5)
    send("STOP")"""