import asyncio
import json
import time

from serial_link import SerialLink
from foxglove_websocket.server import FoxgloveServer

from slam.map import OccupancyGrid
occ_grid = OccupancyGrid(size=300)

print("Starting BORIS Foxglove bridge...")

serial = SerialLink("/dev/ttyUSB0")

def convert_grid_for_foxglove(occ_grid):
    size = occ_grid.size
    resolution = 0.05  # meters per cell

    origin_x = -size * resolution / 2
    origin_y = -size * resolution / 2

    flat_data = occ_grid.grid.flatten().tolist()

    return {
        "resolution": resolution,
        "width": size,
        "height": size,
        "origin": {"x": origin_x, "y": origin_y, "z": 0.0},
        "data": flat_data
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


        slam_channel = await server.add_channel({
            "topic": "boris/slam_map",
            "encoding": "json",
            "schemaName": "foxglove.OccupancyGrid",
            "schemaEncoding": "jsonschema",
            "schema": json.dumps(OCCUPANCY_GRID_SCHEMA)
        })

        print("[BRIDGE] SLAM map channel created")

        count = 0

        while True:
            ultrasonic_cm, imu = serial.read_sensors()

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

            # Send telemetry
            await server.send_message(
                telemetry_channel,
                int(time.time() * 1e9),
                json.dumps(payload).encode("utf-8")
            )

            slam_msg = {
                "timestamp": int(time.time() * 1e9),
                "frame_id": "map",
                "pose": {
                    "position": {"x": 0, "y": 0, "z": 0},
                    "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}
                },
                **convert_grid_for_foxglove(occ_grid)
            }

            await server.send_message(
                slam_channel,
                int(time.time() * 1e9),
                slam_msg
            )


            # Send SLAM map every 10 cycles (2 Hz)
            if count % 10 == 0:
                map_data = convert_grid_for_foxglove(occ_grid)

                await server.send_message(
                    slam_channel,
                    int(time.time() * 1e9),
                    map_data
                    #json.dumps(map_data).encode("utf-8")
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
