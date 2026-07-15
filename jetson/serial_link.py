def _parse_message(self, msg):
    try:
        # Only parse sensor packets
        if not msg.startswith("U:"):
            return

        parts = msg.split(',')
        data = {}

        for p in parts:
            if ':' not in p:
                continue
            key, val = p.split(':', 1)
            data[key] = val

        if "U" in data:
            self.last_ultrasonic = int(data["U"])

        if "Y" in data and "P" in data and "R" in data:
            yaw = float(data["Y"])
            pitch = float(data["P"])
            roll = float(data["R"])
            self.last_imu = (yaw, pitch, roll)

    except Exception as e:
        print(f"[SerialLink] Parse error: {e}")
