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
        "timestamp": {
            "type": "number",
            "description": "Unix timestamp"
        },
        "distance_cm": {
            "type": "number",
            "nullable": True,
            "description": "Ultrasonic distance in cm"
        },
        "yaw": {
            "type": "number",
            "nullable": True,
            "description": "Yaw angle in degrees"
        },
        "pitch": {
            "type": "number",
            "nullable": True,
            "description": "Pitch angle in degrees"
        },
        "roll": {
            "type": "number",
            "nullable": True,
            "description": "Roll angle in degrees"
        }
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
            "schemaEncoding": "jsonschema",
            "schema": json.dumps(BORIS_SCHEMA)
        }
    }

    await websocket.send(json.dumps(channel_def))
    print("[BRIDGE] Channel definition sent")

    count = 0

    while True:
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
            "roll": roll,
        }

        envelope = {
            "op": "message",
            "channelId": channel_id,
            "timestamp": int(time.time() * 1e9)
        }

        try:
            await websocket.send(json.dumps(envelope))

            await websocket.send_bytes(json.dumps(payload).encode('utf-8'))



        except Exception as e:
            print(f"[ERROR] {e}")
            break

        count += 1
        if count % 20 == 0:
            print(f"[BRIDGE] Sent {count} messages  Distance={ultrasonic_cm}")

        await asyncio.sleep(0.05)


loop = asyncio.get_event_loop()
server = websockets.serve(handler, "0.0.0.0", 8765)
loop.run_until_complete(server)
loop.run_forever()
