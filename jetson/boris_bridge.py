import asyncio
import websockets
import json
import time

from serial_link import SerialLink

print("Starting BORIS Foxglove bridge...")

serial = SerialLink("/dev/ttyUSB0")

BORIS_SCHEMA = {
    "type": "object",
    "properties": {
        "timestamp": {"type": "number"},
        "distance_cm": {"type": ["number", "null"]},
        "yaw": {"type": ["number", "null"]},
        "pitch": {"type": ["number", "null"]},
        "roll": {"type": ["number", "null"]}
    }
}

async def handler(websocket, path):
    print("[BRIDGE] Foxglove client connected")

    channel_id = 1

    channel_def = {
        "op": "add_channel",
        "channel": {
            "id": channel_id,
            "topic": "boris/telemetry",
            "encoding": "json",
            "schemaName": "BorisData",
            "schema": json.dumps(BORIS_SCHEMA)
        }
    }

    await websocket.send(json.dumps(channel_def))
    print("[BRIDGE] ✓ Channel definition sent")

    count = 0

    while True:
        try:
            ultrasonic_cm, imu = serial.read_sensors()

            if imu is None:
                yaw = pitch = roll = None
            else:
                yaw, pitch, roll = imu

            payload = {
                "timestamp": time.time(),
                "distance_cm": ultrasonic_cm,
                "yaw": yaw,
                "pitch": pitch,
                "roll": roll
            }

            message = {
                "op": "message",
                "channelId": channel_id,
                "timestamp": int(time.time() * 1e9),
                "message": payload
            }

            await websocket.send(json.dumps(message))

            count += 1
            if count % 20 == 0:
                print(f"[BRIDGE] ✓ Sent {count} messages - Distance={ultrasonic_cm}")

            await asyncio.sleep(0.05)

        except websockets.exceptions.ConnectionClosed:
            print("[BRIDGE] ⚠ Client disconnected")
            break
        except Exception as e:
            print(f"[ERROR] {e}")
            await asyncio.sleep(0.1)

async def main():
    print("[BRIDGE] Starting server on ws://0.0.0.0:8765")
    print("[BRIDGE] Waiting for Foxglove connection...")

    async with websockets.serve(handler, "0.0.0.0", 8765):
        await asyncio.Future()

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
    loop.close()
