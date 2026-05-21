import serial

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

def read_ultrasonic():
    line = ser.readline().decode().strip()
    if line.startswith("ULTRASONIC:"):
       return int(line.split(":")[1])
    return None

def read_line():
    return ser.readline().decode().strip()

def read_imu():
    line = ser.readline().decode().strip()
    if line.startswith("IMU:"):
        parts = line.split(":")[1].split(",")
        return tuple(map(float, parts))
    return None
    # Example: "IMU:1.2,0.1,-0.3" → (1.2, 0.1, -0.3)


def send(command):
    ser.write((command + "\n").encode())
    print(f"Sent command: {command}")

def serialLink():
    print("SerialLink: Not implemented yet")
