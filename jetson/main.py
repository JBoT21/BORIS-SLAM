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




