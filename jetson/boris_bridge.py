import asyncio
import websockets
import json
import time

from serial_link import SerialLink

print("Starting BORIS Foxglove bridge...")

# Open serial connection to ESP32
serial = SerialLink("/dev/ttyUSB0")


async def handler(websocket, path):
    print("Foxglove client connected")

    channel_id = 1

    # Foxglove channel definition
    channel_def = {
        "op": "add_channel",
        "channel": {
            "id": channel_id,
            "topic": "boris/telemetry",
            "encoding": "json",
            "schemaName": "",
            "schema": ""
        }
    }

    await websocket.send(json.dumps(channel_def))

    while True:
        # Read sensors from ESP32
        ultrasonic_cm, imu = serial.read_sensors()

        if imu is None:
            yaw = None
            pitch = None
            roll = None
        else:
            yaw, pitch, roll = imu

        # Build telemetry payload
        payload = {
            "timestamp": time.time(),
            "distance_cm": ultrasonic_cm,
            "yaw": yaw,
            "pitch": pitch,
            "roll": roll,
        }

        # Foxglove message format (binary data required)
        msg = {
            "op": "message",
            "channelId": channel_id,
            "timestamp": int(time.time() * 1e9),
            "data": json.dumps(payload).encode()   # ⭐ FIXED: must be bytes
        }

        try:
            await websocket.send(json.dumps(msg))
        except Exception as e:
            print(f"Error in connection handler: {e}")
            break

        await asyncio.sleep(0.05)  # 50ms update rate


# Start WebSocket server
loop = asyncio.get_event_loop()
server = websockets.serve(handler, "0.0.0.0", 8765)
loop.run_until_complete(server)
loop.run_forever()
