"""
main.py — Jetson Nano SLAM Control Loop
Author: Jackson Bowen
Description:
    High-level SLAM + navigation loop for Jetson Nano.
    Communicates with ESP32 for motors, servo, and sensors.

Psuedocode for right now

init serial
init map
init localization

loop:
    read sensors
    update localization
    update map
    decide next movement
    send movement command
    visualize (optional)

"""
import os
import time
#Hardware interfaces
from serial_link import *
from sensors import SensorParser
from motion import MotionController

#SLAM components
from slam.map import OccupancyGrid
from slam.localization import Localization
from slam.mapping import MappingEngine
from slam.nav import Navigator
from slam.visualize import Visualizer   # future development

def main():
    print("Starting Jetson Nano + ESP32 SLAM System!!!!")
    print("Initializing hardware interfaces...")

    serial = SerialLink(port="/dev/ttyUSB0", baud=115200)
    #Changed to work with UART
    motion = MotionController(serial)

    print("Initializing SLAM components...")
    grid = OccupancyGrid(size=200)
    localization = Localization(grid)
    mapper = MappingEngine(grid, localization)
    navigator = Navigator(grid, localization)
    headless = not os.environ.get("DISPLAY")

    if not headless:
        visual = Visualizer(grid, localization, show_rays=True)
    else:
        print("[Visualizer] Running headless — visualization disabled")

        class DummyVisualizer:
            def update(self):
                pass

        visual = DummyVisualizer()

    print("SLAM components initialized.")
    print("Entering main control loop...")


    LoopHZ = 10
    LoopDT = 1.0 / LoopHZ

    try:
        while True:
            loop_start = time.time()

            # 1. Read sensors (ultrasonic + IMU)
            ultra, imu = serial.read_sensors()
            if ultra is not None:
                mapper.update_from_ultrasonic(ultra)
                localization.last_distance = ultra  # for visualizer
            if imu is not None:
                localization.update_from_imu(imu)

            # 2. Predict localization 
            localization.predict(dt=LoopDT)

            # 3. Update map
            mapper.integrate()

            # 4. Navigation decision
            command = navigator.decide_next_move()
            motion.execute(command)

            # Notify localization of movement
            localization.notify_motion(command, speed=120)

            # 5. Visualization
            visual.update()

            # 6. Loop timing
            elapsed = time.time() - loop_start
            if elapsed < LoopDT:
                time.sleep(LoopDT - elapsed)
            else:
                print(f"[WARN] Loop overran: {elapsed:.3f}s (target {LoopDT:.3f}s)")

    except KeyboardInterrupt:
        print("Shutting down SLAM system...")
        motion.stop()
        serial.close()


if __name__ == "__main__":
    main()