import asyncio
import json
import time

from serial_link import SerialLink
from foxglove_websocket.server import FoxgloveServer

from mapping import MappingEngine

print("Starting BORIS Foxglove bridge...")

serial = SerialLink("/dev/ttyUSB0")
mapper = MappingEngine()  # uses GRID_W x GRID_H, UNKNOWN/FREE/OCCUPIED

def convert_slam_grid_for_foxglove(grid):
    height, width = grid.shape
    resolution = 0.25  # 25 cm per cell

    origin_x = -width * resolution / 2
    origin_y = -height * resolution / 2

    flat = grid.flatten().tolist()

    return {
        "resolution": resolution,
        "width": width,
        "height": height,
        "origin": {"x": origin_x, "y": origin_y, "z": 0.0},
        "data": flat
    }

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

OCCUPANCY_GRID_SCHEMA = {
    "type": "object",
    "properties": {
        "timestamp": {"type": "number"},
        "frame_id": {"type": "string"},
        "pose": {
            "type": "object",
            "properties": {
                "position": {
                    "type": "object",
                    "properties": {
                        "x": {"type": "number"},
                        "y": {"type": "number"},
                        "z": {"type": "number"}
                    }
                },
                "orientation": {
                    "type": "object",
                    "properties": {
                        "x": {"type": "number"},
                        "y": {"type": "number"},
                        "z": {"type": "number"},
                        "w": {"type": "number"}
                    }
                }
            }
        },
        "resolution": {"type": "number"},
        "width": {"type": "number"},
        "height": {"type": "number"},
        "origin": {
            "type": "object",
            "properties": {
                "x": {"type": "number"},
                "y": {"type": "number"},
                "z": {"type": "number"}
            }
        },
        "data": {
            "type": "array",
            "items": {"type": "number"}
        }
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
            "schemaName": "foxglove.OccupancyGrid",
            "schemaEncoding": "jsonschema",
            "schema": json.dumps(OCCUPANCY_GRID_SCHEMA)
        })
        print("[BRIDGE] ✓ SLAM map channel created")

        count = 0

        while True:
            # Read sensors from ESP32
            ultrasonic_cm, imu = serial.read_sensors()

            # If you have a packet for mapping, feed it here:
            # e.g. packet = serial.read_packet()
            # mapper.update_from_packet(packet)

            if imu is not None:
                yaw, pitch, roll = imu
            else:
                yaw = pitch = roll = None

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

            # Send SLAM map every 10 cycles (~2 Hz)
            if count % 10 == 0:
                grid = mapper.get_grid()
                slam_msg = {
                    "timestamp": int(time.time() * 1e9),
                    "frame_id": "map",
                    "pose": {
                        "position": {"x": 0, "y": 0, "z": 0},
                        "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}
                    },
                    **convert_slam_grid_for_foxglove(grid)
                }

                await server.send_message(
                    slam_channel,
                    int(time.time() * 1e9),
                    json.dumps(slam_msg).encode("utf-8")
                )
                print("[BRIDGE] SLAM map sent")

            count += 1

            if count % 20 == 0:
                print(f"[BRIDGE] Sent {count} messages - Distance={ultrasonic_cm}")

            await asyncio.sleep(0.05)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[BRIDGE] Shutting down")
