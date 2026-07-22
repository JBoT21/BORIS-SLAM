import asyncio
import json
import time
import numpy as np
import logging

from serial_link import SerialLink
from foxglove_websocket.server import FoxgloveServer
from slam.mapping import MappingEngine

logging.basicConfig(level=logging.INFO)

print("Starting BORIS Foxglove bridge...")

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
def grid_to_image(grid):
    img = np.zeros_like(grid, dtype=np.uint8)
    img[grid == -1] = 127   # unknown = gray
    img[grid == 0]  = 255   # free = white
    img[grid == 100] = 0    # occupied = black
    return img.flatten().tolist()

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

# Correct foxglove.Image schema (with required "step")
IMAGE_SCHEMA = {
    "type": "object",
    "properties": {
        "timestamp": {"type": "number"},
        "frame_id": {"type": "string"},
        "width": {"type": "number"},
        "height": {"type": "number"},
        "encoding": {"type": "string"},
        "step": {"type": "number"},     
        "data": {
            "type": "array",
            "items": {"type": "number"}
        }
    },
    "required": ["timestamp", "frame_id", "width", "height", "encoding", "step", "data"]
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
            "topic": "boris/telemetry",
            "encoding": "json",
            "schemaName": "BorisData",
            "schemaEncoding": "jsonschema",
            "schema": json.dumps(BORIS_SCHEMA),
        })
        print("[BRIDGE] ✓ Telemetry channel created")

        # SLAM map as foxglove.Image
        slam_channel = await server.add_channel({
            "topic": "boris/slam_map",
            "encoding": "json", 
            "schemaName": "foxglove.Image",
            "schemaEncoding": "jsonschema",
            "schema": json.dumps(IMAGE_SCHEMA),
        })
        print("[BRIDGE] ✓ SLAM image channel created")

        count = 0

        while True:
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

            # SLAM image every 10 cycles (~2 Hz)
            if count % 10 == 0:
                grid = mapper.get_grid().copy()
                grid[grid == 2] = 100  # occupied = 100

                height, width = grid.shape

                image_msg = {
                    "timestamp": int(time.time() * 1e9),
                    "frame_id": "map",
                    "width": width,
                    "height": height,
                    "encoding": "mono8",
                    "step": width,                   
                    "data": grid_to_image(grid),
                }

                await server.send_message(
                    slam_channel,
                    int(time.time() * 1e9),
                    json.dumps(image_msg).encode("utf-8")
                )

                print("[BRIDGE] SLAM image sent")

            count += 1

            if count % 20 == 0:
                print(f"[BRIDGE] Sent {count} messages - Distance={ultrasonic_cm}")

            await asyncio.sleep(0.05)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[BRIDGE] Shutting down")
