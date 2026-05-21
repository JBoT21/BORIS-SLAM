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
import time
#Hardware interfaces
from serial_link import serialLink
from sensors import SensorParser
from motion import MotionController

#SLAM components
from slam.map import OccupancyGrid
from slam.localization import Localization
from slam.mapping import MappingEngine
from slam.nav import Navigator
from slam.visualize import Visualizer   # future development

def main():
    print("Starting Jetson Nano + ESP32 SLAM System!!!!!")
    print("Initializing hardware interfaces...")

    serial = SerialLink(port="/dev/ttyUSB0", baud=115200)
    sensors = SensorParser()
    motion = MotionController(serial)
    print("Hardware interfaces initialized.")

    print("Initializing SLAM components...")
    grid = OccupancyGrid(size = 200)
    localization = Localization(grid)
    mapper = MappingEngine(grid, localization)
    navigator = Navigator(grid, localization)
    visual = Visualizer(grid, localization)  # future development
    print("SLAM components initialized.")

    print("Entering main control loop...")

    #Evaluate these variables to adjust loop timing and performance
    LoopHZ = 10
    #LoopHZ is the frequency at which the main loop runs (10 Hz = 100 ms per iteration)
    LoopDT = 1.0 / LoopHZ
    #LoopDT is the time delay between iterations (0.1 seconds for 10 Hz)

    try:
        while True:
            loop_start = time.time()

            #1. Read sensors
            raw = serial.read_line()
            sensor_data = sensors.parse(raw)

            if sensor_data.ultrasonic is not None:
                mapper.update_from_ultrasonic(sensor_data.ultrasonic)
            if sensor_data.imu is not None:
                localization.update_from_imu(sensor_data.imu)

            #2. Update localization
                localization.predict()

            #3. Update map
                mapper.integrate()

            #4 Navigation decision
                command = navigator.decide_next_move()
                motion.execute(command)

            #5. Visualization (optional)
                visual.update()

            #6. Keep track of loop timing
                time_elapsed = time.time() - loop_start
                if time_elapsed < LoopDT:
                    time.sleep(LoopDT - time_elapsed)
                else:
                    print(f"Warning: Loop took {time_elapsed:.2f} seconds. Desired time is: {LoopDT:.2f} seconds.")

                
    except KeyboardInterrupt:
        print("Shutting down SLAM system...")
        motion.stop()
        serial.close()

if __name__ == "__main__":
    main()




