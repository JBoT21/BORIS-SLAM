from serial_link import read_ultrasonic, send
import numpy as np
import time

MAP_SIZE = 200
grid = np.zeros((MAP_SIZE, MAP_SIZE))

while True:
    distance = read_ultrasonic()
    if distance is not None:
        print(f"Distance: {distance} cm")
        # Update grid based on distance


    """send("MOVE FWD 150")
    time.sleep(0.5)
    send("STOP")"""