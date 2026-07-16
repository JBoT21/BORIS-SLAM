import asyncio
import websockets
import json
import time
import base64
 
from serial_link import SerialLink
 
print("Starting BORIS Foxglove bridge...")
 
serial = SerialLink("/dev/ttyUSB0")
 
# JSON schema for BORIS telemetry (Foxglove format)
BORIS_SCHEMA = json.dumps({
    "type": "object",
    "properties": {
        "timestamp": {"type": "number", "description": "Timestamp in seconds"},
        "distance_cm": {"type": ["number", "null"], "description": "Ultrasonic distance in cm"},
        "yaw": {"type": ["number", "null"], "description": "Yaw angle in degrees"},
        "pitch": {"type": ["number", "null"], "description": "Pitch angle in degrees"},
        "roll": {"type": ["number", "null"], "description": "Roll angle in degrees"},
    }
})
 
async def handler(websocket, path):
    print("[BRIDGE] Foxglove client connected")
 
    channel_id = 1
 
    # Send channel definition
    channel_def = {
    "op": "add_channel",
    "channel": {
        "id": channel_id,
        "topic": "boris/telemetry",
        "encoding": "json",
        "schemaEncoding": "jsonschema",
        "schemaName": "BorisData",
        "schema": BORIS_SCHEMA,
    },
}
    
    try:
        await websocket.send(json.dumps(channel_def))
        print("[BRIDGE]  Channel definition sent")
    except Exception as e:
        print(f"[ERROR] Failed to send channel definition: {e}")
        return
 
    message_count = 0
    error_count = 0
    
    while True:
        try:
            # Read sensor data from ESP32
            ultrasonic_cm, imu = serial.read_sensors()
 
            if imu is None:
                yaw, pitch, roll = None, None, None
            else:
                yaw, pitch, roll = imu
 
            # Create data payload
            payload = {
                "timestamp": time.time(),
                "distance_cm": ultrasonic_cm,
                "yaw": yaw,
                "pitch": pitch,
                "roll": roll,
            }
 
            # Send message in Foxglove format
            message = {
                "op": "message",
                "channelId": channel_id,
                "timestamp": int(time.time() * 1e9),  # Nanoseconds
                "data": base64.b64encode(json.dumps(payload).encode()).decode()
            }
 
            await websocket.send(json.dumps(message))
            message_count += 1
            
            # Print progress (every 20 messages)
            if message_count % 20 == 0:
                status = f"Distance: {ultrasonic_cm}cm"
                if yaw is not None:
                    status += f", Yaw: {yaw:.1f}°"
                print(f"[BRIDGE] ✓ Sent {message_count} messages - {status}")
 
            await asyncio.sleep(0.05)  # 20Hz update rate
 
        except websockets.exceptions.ConnectionClosed:
            print("[BRIDGE] ⚠ Client disconnected")
            break
        except Exception as e:
            error_count += 1
            print(f"[ERROR] {error_count}: {e}")
            if error_count > 10:
                print("[ERROR] Too many errors, breaking connection")
                break
            await asyncio.sleep(0.1)
 
 
async def main():
    print("[BRIDGE] Starting server on ws://0.0.0.0:8765")
    print("[BRIDGE] Make sure Foxglove is configured to connect to this address")
    
    async with websockets.serve(handler, "0.0.0.0", 8765):
        print("[BRIDGE] Server running, waiting for Foxglove connection...")
        await asyncio.Future()  # Run forever
 
 
if __name__ == "__main__":
    try:
        loop = asyncio.get_event_loop()
        loop.run_until_complete(main())
    except KeyboardInterrupt:
        print("\n[BRIDGE] Shutting down...")
    except Exception as e:
        print(f"[ERROR] Fatal: {e}")
 