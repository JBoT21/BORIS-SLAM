import serial
import time

class SerialLink:
    """
    Handles UART communication between Jetson Nano and ESP32.
    Combined packet format:
        U:<cm>,Y:<yaw>,P:<pitch>,R:<roll>
    """

    def __init__(self, port="/dev/ttyUSB0", baud=115200, timeout=0.1):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(2)

        self.last_ultrasonic = None
        self.last_imu = None

        print(f"[SerialLink] Connected on {port} @ {baud}")

    def read_line(self):
        try:
            raw = self.ser.readline().decode(errors="ignore").strip()
            if not raw:
                return None

            # Filter ESP32 boot garbage
            if "rst:" in raw or "boot:" in raw:
                return None

            return raw

        except Exception as e:
            print(f"[SerialLink] Read error: {e}")
            return None

    def _parse_message(self, msg):
        try:
            parts = msg.split(',')
            data = {}

            for p in parts:
                key, val = p.split(':')
                data[key] = val

            # Extract ultrasonic
            if "U" in data:
                self.last_ultrasonic = int(data["U"])

            # Extract IMU
            if "Y" in data and "P" in data and "R" in data and "H" in data:
                yaw = float(data["Y"])
                pitch = float(data["P"])
                roll = float(data["R"])
                heading_index = int(data["H"])
                self.last_imu = (yaw, pitch, roll, heading_index)

        except Exception as e:
            print(f"[SerialLink] Parse error: {e}")

    def read_sensors(self):
      #Reads a single combined packet
        line = self.read_line()
        if line:
            self._parse_message(line)

        return self.last_ultrasonic, self.last_imu

    def send(self, command):
        try:
            self.ser.write((command + "\n").encode())
        except Exception as e:
            print(f"[SerialLink] Send error: {e}")

    def close(self):
        self.ser.close()
        print("[SerialLink] Closed serial port")
