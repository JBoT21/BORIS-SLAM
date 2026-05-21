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
from sensors import UltrasonicParser, IMUParser
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
    mapping = MappingEngine(grid, localization)
    navigator = Navigator(grid, localization)
    visual = Visualizer(grid, localization)  # future development
    print("SLAM components initialized.")

    print("Entering main control loop...")

    #Evaluate these variables to adjust loop timing and performance
    LoopHZ = 10
    #LoopHZ is the frequency at which the main loop runs (10 Hz = 100 ms per iteration)
    LoopDT = 1.0 / LoopHZ
    #LoopDT is the time delay between iterations (0.1 seconds for 10 Hz)



