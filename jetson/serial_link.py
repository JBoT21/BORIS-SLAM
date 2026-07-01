import serial
import time

class SerialLink:
    """
    Handles UART communication between Jetson Nano and ESP32.
    Parsing format:
        - ULTRASONIC:<cm>
        - IMU:<yaw>,<pitch>,<roll>
    """

    def __init__(self, port="/dev/ttyUSB0", baud=115200, timeout=0.1):
        self.ser = serial.Serial(port, baud, timeout=timeout, rtscts=False, dsrdtr=False, xonxoff=False)
        time.sleep(2)  # allows the ESP32 to boot

        self.last_ultrasonic = None
        self.last_imu = None

        print(f"[SerialLink] Connected on {port} @ {baud}")

    # Low-level read
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

    # Parse a single message
    def _parse_message(self, msg):
        if msg.startswith("ULTRASONIC:"):
            try:
                self.last_ultrasonic = int(msg.split(":")[1])
            except:
                pass
        elif msg.startswith("IMU:"):
            try:
                parts = msg.split(":")[1].split(",")
                yaw, pitch, roll = map(float, parts)
                self.last_imu = (yaw, pitch, roll)
            except:
                pass

    # Read and parse all available sensor messages
    def read_sensors(self):
        line = self.read_line()
        if not line:
            return self.last_ultrasonic, self.last_imu

        # Handle multiple messages in one line (it happens :/)
        for msg in line.split("\n"):
            msg = msg.strip()
            if msg:
                self._parse_message(msg)

        return self.last_ultrasonic, self.last_imu

    def send(self, command):
        try:
            self.ser.write((command + "\n").encode())
        except Exception as e:
            print(f"[SerialLink] Send error: {e}")

    def close(self):
        self.ser.close()
        print("[SerialLink] Closed serial port")