import asyncio
import json
import time
import numpy as np
import logging
import base64

from serial_link import SerialLink
from foxglove_websocket.server import FoxgloveServer
from slam.mapping import MappingEngine

logging.basicConfig(level=logging.INFO)

print("Starting BORIS Foxglove bridge with RawImage...")

serial = SerialLink("/dev/ttyUSB0")
mapper = MappingEngine()

# Convert yaw (degrees) to heading index
def heading_from_yaw(yaw):
    yaw = yaw % 360
    if yaw < 45 or yaw >= 315:
        return 0
    elif yaw < 135:
        return 1
    elif yaw < 225:
        return 2
    else:
        return 3

# Convert SLAM grid → grayscale mono8 image
def grid_to_image_bytes(grid):
    """Convert occupancy grid to mono8 image bytes"""
    img = np.zeros_like(grid, dtype=np.uint8)
    img[grid == -1] = 127   # unknown = gray
    img[grid == 0]  = 255   # free = white
    img[grid == 100] = 0    # occupied = black
    return img.flatten().tobytes()

# Telemetry schema
BORIS_SCHEMA = {
    "type": "object",
    "properties": {
        "timestamp": {"type": "number"},
        "distance_cm": {"type": ["number", "null"]},
        "yaw": {"type": ["number", "null"]},
        "pitch": {"type": ["number", "null"]},
        "roll": {"type": ["number", "null"]},
    }
}

# RawImage schema
IMAGE_SCHEMA = {
    "type": "object",
    "properties": {
        "timestamp": {"type": "number"},
        "frame_id": {"type": "string"},
        "width": {"type": "integer"},
        "height": {"type": "integer"},
        "encoding": {"type": "string"},
        "data": {"type": "string"}  # base64
    }
}

async def main():

    async with FoxgloveServer(
        host="0.0.0.0",
        port=8765,
        name="BORIS SLAM"
    ) as server:

        print("[BRIDGE] Foxglove server running")
        print("[BRIDGE] Connect to ws://<jetson-ip>:8765")

        # Telemetry channel
        telemetry_channel = await server.add_channel({
            "topic": "/boris/telemetry",
            "encoding": "json",
            "schemaName": "BorisData",
            "schemaEncoding": "jsonschema",
            "schema": json.dumps(BORIS_SCHEMA),
        })
        print("[BRIDGE] ✓ Telemetry channel created")

        # RawImage channel
        image_channel = await server.add_channel({
            "topic": "/boris/map",
            "encoding": "json", 
            "schemaName": "foxglove.RawImage",
            "schemaEncoding": "jsonschema",
            "schema": json.dumps(IMAGE_SCHEMA),
        })
        print("[BRIDGE] ✓ RawImage channel created")

        count = 0

        while True:
            try:
                ultrasonic_cm, imu = serial.read_sensors()

                # Telemetry IMU
                if imu is not None:
                    yaw, pitch, roll = imu
                else:
                    yaw = pitch = roll = None

                # SLAM update
                if ultrasonic_cm is not None and imu is not None:
                    heading_index = heading_from_yaw(yaw)
                    packet = f"U:{ultrasonic_cm},H:{heading_index}"
                    mapper.update_from_packet(packet)

                # Telemetry payload
                payload = {
                    "timestamp": time.time(),
                    "distance_cm": ultrasonic_cm,
                    "yaw": yaw,
                    "pitch": pitch,
                    "roll": roll,
                }

                await server.send_message(
                    telemetry_channel,
                    int(time.time() * 1e9),
                    json.dumps(payload).encode("utf-8")
                )

                # RawImage every 10 cycles (~2 Hz)
                if count % 10 == 0:
                    grid = mapper.get_grid().copy()
                    
                    if len(grid.shape) != 2:
                        print(f"[ERROR] Grid has unexpected shape: {grid.shape}")
                    else:
                        height, width = grid.shape

                        # Convert to mono8 image bytes
                        image_bytes = grid_to_image_bytes(grid)
                        
                        # Base64 encode
                        image_base64 = base64.b64encode(image_bytes).decode('utf-8')

                        # Create RawImage message
                        image_msg = {
                            "timestamp": time.time(),
                            "frame_id": "map",
                            "width": width,
                            "height": height,
                            "encoding": "mono8",
                            "data": image_base64,
                        }

                        await server.send_message(
                            image_channel,
                            int(time.time() * 1e9),
                            json.dumps(image_msg).encode("utf-8")
                        )

                        print(f"[BRIDGE] ✓ RawImage sent ({width}x{height})")

                count += 1

                if count % 20 == 0:
                    print(f"[BRIDGE] {count} messages - Distance={ultrasonic_cm}cm")

                await asyncio.sleep(0.05)

            except Exception as e:
                print(f"[ERROR] {e}")
                import traceback
                traceback.print_exc()
                await asyncio.sleep(0.1)


def main_wrapper():
    """Wrapper for Python 3.6 compatibility"""
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(main())
    except KeyboardInterrupt:
        print("\n[BRIDGE] Shutting down...")
    finally:
        loop.close()


if __name__ == "__main__":
    main_wrapper()