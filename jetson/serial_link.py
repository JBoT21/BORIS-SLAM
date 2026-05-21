import serial
import time

class SerialLink:
    """
    Handles UART communication between Jetson Nano and ESP32.
    Provides:
        - read_line()
        - read_ultrasonic()
        - read_imu()
        - send()
        - close()
    """

    def __init__(self, port="/dev/ttyUSB0", baud=115200, timeout=0.1):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(2)  # allow ESP32 to reset
        print(f"[SerialLink] Connected on {port} @ {baud}")

    def read_line(self):
        try:
            line = self.ser.readline().decode(errors="ignore").strip()
            if line:
                return line
        except Exception as e:
            print(f"[SerialLink] Read error: {e}")
        return None

    def read_ultrasonic(self, line):
        if line and line.startswith("ULTRASONIC:"):
            try:
                return int(line.split(":")[1])
            except:
                return None
        return None

    def read_imu(self, line):
        if line and line.startswith("IMU:"):
            try:
                parts = line.split(":")[1].split(",")
                yaw, pitch, roll = map(float, parts)
                return (yaw, pitch, roll)
            except:
                return None
        return None

    def send(self, command):
        try:
            self.ser.write((command + "\n").encode())
        except Exception as e:
            print(f"[SerialLink] Send error: {e}")

    def close(self):
        self.ser.close()
        print("[SerialLink] Closed serial port")
