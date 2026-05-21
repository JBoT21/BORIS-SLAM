"""
Parses raw strings from serial_link.py into structured data.
Responsibilities:

1. ex: Convert "ULTRASONIC:42" → 42

2. ex: Convert "IMU:1.2,0.1,-0.3" → (yaw, pitch, roll)

3. Validate ranges?

4. Smooth noisy readings
"""
def SensorParser(rawStr):
    if rawStr.startswith("ULTRASONIC:"):
        return UltrasonicParser(rawStr)
    elif rawStr.startswith("IMU:"):
        return IMUParser(rawStr)
    else:
        print(f"Unknown sensor type in string: {rawStr}")
        return None
    

def UltrasonicParser(rawStr):
    if rawStr.startswith("ULTRASONIC:"):
        try:
            return int(rawStr.split(":")[1])
        except ValueError:
            print(f"Invalid ultrasonic reading: {rawStr}")
            return None
    else:
        print(f"Unexpected format for ultrasonic reading: {rawStr}")
        return None
    

def IMUParser(rawStr):
    if rawStr.startswith("IMU:"):
        try:
            parts = rawStr.split(":")[1].split(",")
            return tuple(map(float, parts))
        except ValueError:
            print(f"Invalid IMU reading: {rawStr}")
            return None
    else:
        print(f"Unexpected format for IMU reading: {rawStr}")
        return None
    