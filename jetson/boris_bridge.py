import asyncio
import json
import time
import numpy as np
import logging
 
from serial_link import SerialLink
from foxglove_websocket.server import FoxgloveServer
from slam.mapping import MappingEngine
 
logging.basicConfig(level=logging.INFO)
 
print("Starting BORIS Foxglove bridge with OccupancyGrid...")
 
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
 
# Foxglove OccupancyGrid schema (ROS standard)
OCCUPANCY_GRID_SCHEMA = {
    "type": "object",
    "properties": {
        "timestamp": {"type": "number"},
        "frame_id": {"type": "string"},
        "info": {
            "type": "object",
            "properties": {
                "map_load_time": {"type": "number"},
                "resolution": {"type": "number"},
                "width": {"type": "integer"},
                "height": {"type": "integer"},
                "origin": {
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
                }
            }
        },
        "data": {
            "type": "array",
            "items": {"type": "integer"}
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
 
        # Telemetry channel
        telemetry_channel = await server.add_channel({
            "topic": "boris/telemetry",
            "encoding": "json",
            "schemaName": "BorisData",
            "schemaEncoding": "jsonschema",
            "schema": json.dumps(BORIS_SCHEMA),
        })
        print("[BRIDGE] ✓ Telemetry channel created")
 
        # OccupancyGrid channel
        grid_channel = await server.add_channel({
            "topic": "boris/map",
            "encoding": "json", 
            "schemaName": "nav_msgs.OccupancyGrid",  # ← Foxglove built-in type
            "schemaEncoding": "jsonschema",
            "schema": json.dumps(OCCUPANCY_GRID_SCHEMA),
        })
        print("[BRIDGE] ✓ OccupancyGrid channel created")
 
        count = 0
        resolution = 0.05  # meters per cell (5cm)
 
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
 
                # OccupancyGrid every 10 cycles (~2 Hz)
                if count % 10 == 0:
                    grid = mapper.get_grid().copy()
                    
                    if len(grid.shape) != 2:
                        print(f"[ERROR] Grid has unexpected shape: {grid.shape}")
                    else:
                        height, width = grid.shape
                        
                        # Convert grid values to ROS occupancy values (-1 to 100)
                        # Grid values: 0=unknown, 1=free, 2=occupied
                        occupancy_data = []
                        for value in grid.flatten():
                            if value == 1:
                                occupancy_data.append(0)      # Free = 0
                            elif value == 2:
                                occupancy_data.append(100)    # Occupied = 100
                            else:
                                occupancy_data.append(-1)     # Unknown = -1
 
                        # Calculate map origin (center of grid)
                        origin_x = -(width * resolution) / 2
                        origin_y = -(height * resolution) / 2
 
                        # Create OccupancyGrid message
                        grid_msg = {
                            "timestamp": time.time(),
                            "frame_id": "map",
                            "info": {
                                "map_load_time": time.time(),
                                "resolution": resolution,
                                "width": width,
                                "height": height,
                                "origin": {
                                    "position": {
                                        "x": origin_x,
                                        "y": origin_y,
                                        "z": 0.0
                                    },
                                    "orientation": {
                                        "x": 0.0,
                                        "y": 0.0,
                                        "z": 0.0,
                                        "w": 1.0
                                    }
                                }
                            },
                            "data": occupancy_data
                        }
 
                        await server.send_message(
                            grid_channel,
                            int(time.time() * 1e9),
                            json.dumps(grid_msg).encode("utf-8")
                        )
 
                        print(f"[BRIDGE] ✓ OccupancyGrid sent ({width}x{height})")
 
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