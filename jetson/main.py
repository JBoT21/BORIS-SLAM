import serial
import time

from slam.mapping import MappingEngine
from slam.nav import Navigator
from slam.visualize import Visualizer   

# Adjust to your Jetson's serial device
SERIAL_PORT = "/dev/ttyUSB0"
BAUD = 115200

def parse_packet(line):
    """
    Packet format:
        U:<dist>,H:<heading_index>
    """
    try:
        parts = line.strip().split(",")
        dist = float(parts[0].split(":")[1])
        heading = int(parts[1].split(":")[1])
        return dist, heading
    except Exception as e:
        print("[main] Parse error:", e, "line:", line)
        return None, None

def main():
    print("[main] Starting BORIS SLAM-Lite...")

    ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)

    mapper = MappingEngine()
    visualizer = Visualizer(mapper)

    while True:
        try:
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                continue

            dist, heading_index = parse_packet(line)
            if dist is None:
                continue

            # Update occupancy grid
            mapper.update_from_packet(line)

            # Update visualizer
            visualizer.update(heading_index)

        except KeyboardInterrupt:
            print("\n[main] Shutting down.")
            break
        except Exception as e:
            print("[main] Error:", e)
            time.sleep(0.1)

if __name__ == "__main__":
    main()




"""
main.py — Jetson Nano SLAM Control Loop
Author: Jackson Bowen
Description:
    High-level SLAM + navigation loop for Jetson Nano.
    Communicates with ESP32 for motors, servo, and sensors.

loop:
    read sensors
    update localization
    update map
    decide next movement
    send movement command (with kalman filter prediction)
    visualize (optional)


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
from slam.visualize import Visualizer   

def main():
    print("Starting Jetson Nano + ESP32 SLAM System!!!!")
    print("Initializing hardware interfaces...")

    serial = SerialLink(port="/dev/ttyUSB0", baud=115200)
    #Changed to work with UART
    motion = MotionController(serial)

    print("Initializing SLAM components...")
    grid = OccupancyGrid(size=20)
    localization = Localization(grid)
    mapper = MappingEngine(grid, localization, max_range=150)  # max range in cm
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
        
    visual = Visualizer(grid, localization, show_rays=True, save_frames=False)


    print("SLAM components initialized.")
    print("Entering main control loop...")


    LoopHZ = 10
    LoopDT = 1.0 / LoopHZ

    try:
        print("LOOP TICK")
        while True:
            loop_start = time.time()
            print("Loop start")

            # 1. Read sensors (ultrasonic + IMU)
            ultra, imu = serial.read_sensors()
            print("[SENSORS DEBUG] ultra =", ultra, "imu =", imu)
            if ultra is not None:
                mapper.update_from_ultrasonic(ultra)
                print(f"[ULTRA DEBUG] Ultrasonic reading: {ultra} cm")
                localization.last_distance = ultra  # for visualizer
            if imu is not None:
                localization.update_from_imu(imu)

            # 2. Predict localization 
            localization.predict(dt=LoopDT)
            print("Localization predicted")

            # 3. Update map
            mapper.integrate()
            print("Map updated")

            # 4. Navigation decision
            command = navigator.decide_next_move()
            print(f"[CMD] Sending command to ESP32: {command}", flush=True)
            motion.execute(command)
            print("Motion commanded")

            # Notify localization of movement
            localization.notify_motion(command, speed=120)
            print("Localization notified of motion")

            # 5. Visualization
            visual.update()
            print("Visualization updated")

            # 6. Loop timing
            elapsed = time.time() - loop_start
            if elapsed < LoopDT:
                time.sleep(LoopDT - elapsed)
            else:
                print(f"[WARN] Loop overran: {elapsed:.3f}s (target {LoopDT:.3f}s)")


    #Was KeyboardInterrupt
    except Exception as e:
        print("ERROR:", e)
        motion.stop()
        serial.close()
        raise


if __name__ == "__main__":
    main()
"""
