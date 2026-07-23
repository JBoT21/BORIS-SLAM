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

print("Starting BORIS Foxglove bridge (RawImage, mono8)...")

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

# Convert SLAM grid → grayscale mono8 image bytes
def grid_to_image_bytes(grid):
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

# Correct foxglove.RawImage schema
RAW_IMAGE_SCHEMA = {
    "type": "object",
    "properties": {
        "timestamp": {"type": "number"},
        "frame_id": {"type": "string"},
        "width": {"type": "integer"},
        "height": {"type": "integer"},
        "encoding": {"type": "string"},      # e.g. "mono8"
        "pixel_format": {"type": "integer"}, # enum value
        "row_stride": {"type": "integer"},   # bytes per row
        "data": {"type": "string"}           # base64-encoded bytes
    },
    "required": [
        "timestamp",
        "frame_id",
        "width",
        "height",
        "encoding",
        "pixel_format",
        "row_stride",
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
            "schemaName": "foxglove.RawImage",
            "schemaEncoding": "jsonschema",
            "schema": json.dumps(RAW_IMAGE_SCHEMA),
        })
        print("[BRIDGE] ✓ SLAM RawImage channel created")

        count = 0
        first_image_sent = False

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

                    print("\n[DEBUG GRID]")
                    print(f"  Shape: {grid.shape}")
                    print(f"  Size: {grid.size}")
                    print(f"  Dtype: {grid.dtype}")
                    print(f"  Min/Max: {grid.min()} / {grid.max()}")
                    print(f"  Unique values: {np.unique(grid)}")

                    if len(grid.shape) != 2:
                        print(f"[ERROR] Grid is not 2D! Shape: {grid.shape}")
                        await asyncio.sleep(0.05)
                        count += 1
                        continue

                    height, width = grid.shape
                    print(f"  Dimensions: {width}x{height}")

                    image_bytes = grid_to_image_bytes(grid)

                    print("[DEBUG IMAGE]")
                    print(f"  Image bytes length: {len(image_bytes)}")
                    print(f"  Expected length: {width * height}")
                    print(f"  Match: {len(image_bytes) == width * height}")

                    image_base64 = base64.b64encode(image_bytes).decode("utf-8")

                    print(f"  Base64 length: {len(image_base64)}")
                    print(f"  First 50 chars: {image_base64[:50]}")

                    # pixel_format: assume 1 = MONO8 (per foxglove enum)
                    image_msg = {
                        "timestamp": time.time(),
                        "frame_id": "map",
                        "width": width,
                        "height": height,
                        "encoding": "mono8",
                        "pixel_format": 1,
                        "row_stride": width,
                        "data": image_base64,
                    }

                    print("[DEBUG MESSAGE]")
                    print(f"  Width: {image_msg['width']}")
                    print(f"  Height: {image_msg['height']}")
                    print(f"  Encoding: {image_msg['encoding']}")
                    print(f"  Pixel format: {image_msg['pixel_format']}")
                    print(f"  Row stride: {image_msg['row_stride']}")
                    print(f"  Data size: {len(image_msg['data'])} chars\n")

                    await server.send_message(
                        slam_channel,
                        int(time.time() * 1e9),
                        json.dumps(image_msg).encode("utf-8")
                    )

                    if not first_image_sent:
                        print("[BRIDGE] ✓ First image sent!")
                        first_image_sent = True

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
