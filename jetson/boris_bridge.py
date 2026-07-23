import asyncio
import json
import time
import numpy as np
import logging

from serial_link import SerialLink
from foxglove_websocket.server import FoxgloveServer
from slam.mapping import MappingEngine

logging.basicConfig(level=logging.INFO)

print("Starting BORIS Foxglove bridge (foxglove.Image, mono8)...")

serial = SerialLink("/dev/ttyUSB0")
mapper = MappingEngine()

def heading_from_yaw(yaw: float) -> int:
    yaw = yaw % 360
    if yaw < 45 or yaw >= 315:
        return 0
    elif yaw < 135:
        return 1
    elif yaw < 225:
        return 2
    else:
        return 3

def grid_to_image_bytes(grid: np.ndarray):
    img = np.zeros_like(grid, dtype=np.uint8)
    img[grid == -1] = 127   # unknown = gray
    img[grid == 0]  = 255   # free = white
    img[grid == 100] = 0    # occupied = black
    return img.flatten().tolist()  # list of ints for JSON

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
    "required": [
        "timestamp",
        "frame_id",
        "width",
        "height",
        "encoding",
        "step",
        "data"
    ]
}

async def main():

    async with FoxgloveServer(
        host="0.0.0.0",
        port=8765,
        name="BORIS SLAM"
    ) as server:

        print("[BRIDGE] Foxglove server running")
        print("[BRIDGE] Connect to ws://<pi-ip>:8765")

        telemetry_channel = await server.add_channel({
            "topic": "boris/telemetry",
            "encoding": "json",
            "schemaName": "BorisData",
            "schemaEncoding": "jsonschema",
            "schema": json.dumps(BORIS_SCHEMA),
        })
        print("[BRIDGE] ✓ Telemetry channel created")

        slam_channel = await server.add_channel({
            "topic": "boris/slam_map",
            "encoding": "json",
            "schemaName": "foxglove.Image",
            "schemaEncoding": "jsonschema",
            "schema": json.dumps(IMAGE_SCHEMA),
        })
        print("[BRIDGE] ✓ SLAM Image channel created")

        count = 0

        while True:
            try:
                ultrasonic_cm, imu = serial.read_sensors()

                if imu is not None:
                    yaw, pitch, roll = imu
                else:
                    yaw = pitch = roll = None

                if ultrasonic_cm is not None and imu is not None:
                    heading_index = heading_from_yaw(yaw)
                    packet = f"U:{ultrasonic_cm},H:{heading_index}"
                    mapper.update_from_packet(packet)

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

                if count % 10 == 0:
                    grid = mapper.get_grid().copy()
                    grid[grid == 2] = 100  # occupied → 100

                    if len(grid.shape) != 2:
                        print(f"[ERROR] Grid is not 2D! Shape: {grid.shape}")
                        await asyncio.sleep(0.05)
                        count += 1
                        continue

                    height, width = grid.shape

                    image_data = grid_to_image_bytes(grid)

                    image_msg = {
                        "timestamp": int(time.time() * 1e9),
                        "frame_id": "map",
                        "width": float(width),
                        "height": float(height),
                        "encoding": "mono8",
                        "step": float(width),   # bytes per row
                        "data": image_data,
                    }

                    await server.send_message(
                        slam_channel,
                        int(time.time() * 1e9),
                        json.dumps(image_msg).encode("utf-8")
                    )

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
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(main())
    except KeyboardInterrupt:
        print("\n[BRIDGE] Shutting down...")
    finally:
        loop.close()


if __name__ == "__main__":
    main_wrapper()
