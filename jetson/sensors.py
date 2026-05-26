"""
Parses raw strings from serial_link.py into structured data.
Responsibilities:

1. ex: Convert "ULTRASONIC:42" → 42

2. ex: Convert "IMU:1.2,0.1,-0.3" → (yaw, pitch, roll)

3. Validate ranges?

4. Smooth noisy readings
"""
"""
sensors.py — Parse raw sensor strings into structured data.
"""

from collections import deque



class SensorData:
    def __init__(self, ultrasonic=None, imu=None):
        self.ultrasonic = ultrasonic
        self.imu = imu


class SensorParser:
    """
    Thin wrapper around SerialLink.
    Converts (ultra, imu) into a SensorData object.
    """

    def __init__(self, serial_link):
        self.serial = serial_link

    def read(self):
        ultra, imu = self.serial.read_sensors()
        return SensorData(ultrasonic=ultra, imu=imu)