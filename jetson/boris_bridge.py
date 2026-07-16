import asyncio
import websockets
import json
import time

from serial_link import SerialLink

print("Starting BORIS Foxglove bridge...")

serial = SerialLink("/dev/ttyUSB0")


async def handler(websocket, path):
    print("Foxglove client connected")

    channel_id = 1

    # Send channel definition (JSON only)
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
        ultrasonic_cm, imu = serial.read_sensors()

        if imu is None:
            yaw = None
            pitch = None
            roll = None
        else:
            yaw, pitch, roll = imu

        payload = {
            "timestamp": time.time(),
            "distance_cm": ultrasonic_cm,
            "yaw": yaw,
            "pitch": pitch,
            "roll": roll,
        }

        # Envelope (JSON)
        envelope = {
            "op": "message",
            "channelId": channel_id,
            "timestamp": int(time.time() * 1e9)
        }

        try:
            # 1) Send envelope as JSON
            await websocket.send(json.dumps(envelope))

            # 2) Send payload as raw bytes (binary frame)
            await websocket.send(json.dumps(payload).encode())

        except Exception as e:
            print(f"Error in connection handler: {e}")
            break

        await asyncio.sleep(0.05)


loop = asyncio.get_event_loop()
server = websockets.serve(handler, "0.0.0.0", 8765)
loop.run_until_complete(server)
loop.run_forever()
