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
import statistics


class SensorData:
    """Container for parsed sensor readings."""
    def __init__(self):
        self.ultrasonic = None
        self.imu = None   # (yaw, pitch, roll)


class SensorParser:
  
    def __init__(self, smooth_window=5):
        self.smooth_window = smooth_window
        self.ultra_history = deque(maxlen=smooth_window)

    def parse(self, raw):
        data = SensorData()

        if raw is None or raw == "":
            return data

        if raw.startswith("ULTRASONIC:"):
            data.ultrasonic = self.parse_ultrasonic(raw)

        elif raw.startswith("IMU:"):
            data.imu = self._parse_imu(raw)

        else:
            print(f"[SensorParser] Unknown sensor string: {raw}")

        return data

    def parse_ultrasonic(self, raw):
        try:
            dist = int(raw.split(":")[1])

            # Value valudation
            if dist < 0 or dist > 500:
                return None

            # Add to history for smoothing
            self.ultra_history.append(dist)
            # Return smoothed value
            return int(statistics.mean(self.ultra_history))

        except Exception:
            print(f"[SensorParser] Invalid ultrasonic reading: {raw}")
            return None
        
    def _parse_imu(self, raw):
        try:
            parts = raw.split(":")[1].split(",")
            yaw, pitch, roll = map(float, parts)
            return (yaw, pitch, roll)

        except Exception:
            print(f"[SensorParser] Invalid IMU reading: {raw}")
            return None
