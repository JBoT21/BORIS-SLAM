import asyncio
import websockets
import json
import time

from serial_link import SerialLink

print("Starting BORIS Foxglove bridge...")

serial = SerialLink("/dev/ttyUSB0")

# JSON schema for BORIS telemetry (Foxglove format)
BORIS_SCHEMA = {
    "type": "object",
    "properties": {
        "timestamp": {"type": "number", "description": "Timestamp in seconds"},
        "distance_cm": {"type": ["number", "null"], "description": "Ultrasonic distance in cm"},
        "yaw": {"type": ["number", "null"], "description": "Yaw angle in degrees"},
        "pitch": {"type": ["number", "null"], "description": "Pitch angle in degrees"},
        "roll": {"type": ["number", "null"], "description": "Roll angle in degrees"},
    }
}


async def handler(websocket, path):
    print("[BRIDGE] Foxglove client connected")

    channel_id = 1

    # Correct Foxglove channel definition
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

    try:
        await websocket.send(json.dumps(channel_def))
        print("[BRIDGE] ✓ Channel definition sent")
    except Exception as e:
        print(f"[ERROR] Failed to send channel definition: {e}")
        return

    message_count = 0

    while True:
        try:
            ultrasonic_cm, imu = serial.read_sensors()

            if imu is None:
                yaw, pitch, roll = None, None, None
            else:
                yaw, pitch, roll = imu

            payload = {
                "timestamp": time.time(),
                "distance_cm": ultrasonic_cm,
                "yaw": yaw,
                "pitch": pitch,
                "roll": roll,
            }

            # 1) Envelope (text frame)
            envelope = {
                "op": "message",
                "channelId": channel_id,
                "timestamp": int(time.time() * 1e9)
            }
            await websocket.send(json.dumps(envelope))

            # 2) Payload (binary frame)
            await websocket.write_message(json.dumps(payload).encode("utf-8"), binary=True)

            message_count += 1

            if message_count % 20 == 0:
                print(f"[BRIDGE] ✓ Sent {message_count} messages - Distance={ultrasonic_cm}")

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
        await asyncio.Future()  # Run forever


if __name__ == "__main__":
    try:
        loop = asyncio.get_event_loop()
        loop.run_until_complete(main())
    except KeyboardInterrupt:
        print("\n[BRIDGE] Shutting down...")
    except Exception as e:
        print(f"[ERROR] Fatal: {e}")
    finally:
        loop.close()
