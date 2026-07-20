import asyncio
import json
import time

from serial_link import SerialLink
#from foxglove_websocket.server import FoxgloveServer
import websockets


print("Starting BORIS Foxglove bridge...")


serial = SerialLink("/dev/ttyUSB0")


BORIS_SCHEMA = {
    "type": "object",
    "properties": {
        "timestamp": {
            "type": "number"
        },
        "distance_cm": {
            "type": ["number", "null"]
        },
        "yaw": {
            "type": ["number", "null"]
        },
        "pitch": {
            "type": ["number", "null"]
        },
        "roll": {
            "type": ["number", "null"]
        },
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


        channel_id = await server.add_channel(
            {
                "topic": "boris/telemetry",
                "encoding": "json",
                "schemaName": "BorisData",
                "schemaEncoding": "jsonschema",
                "schema": json.dumps(BORIS_SCHEMA),
            }
        )


        print("[BRIDGE] Channel created")


        count = 0

        while True:

            ultrasonic_cm, imu = serial.read_sensors()


            if imu is not None:
                yaw, pitch, roll = imu
            else:
                yaw = None
                pitch = None
                roll = None


            payload = {
                "timestamp": time.time(),
                "distance_cm": ultrasonic_cm,
                "yaw": yaw,
                "pitch": pitch,
                "roll": roll,
            }


            await server.send_message(
                channel_id,
                int(time.time() * 1e9),
                json.dumps(payload).encode("utf-8")
            )


            count += 1

            if count % 20 == 0:
                print(
                    f"[BRIDGE] Sent {count} messages "
                    f"Distance={ultrasonic_cm}"
                )


            await asyncio.sleep(0.05)



if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(main())
    except KeyboardInterrupt:
        print("\n[BRIDGE] Shutting down")
    finally:
        loop.close()
