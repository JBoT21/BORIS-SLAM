"""
Responsibilities:

High-level movement commands sent to the ESP32.
This module abstracts away the serial commands

"""

class MotionController:
    def __init__(self, serial_link, localization=None):
        self.serial = serial_link
        self.localization = localization

        # Allowed single-character commands
        self.valid_commands = {"F", "B", "L", "R", "S"}

    # Basic movement commands
    def forward(self, speed=120):
        self._send("F", speed)

    def backward(self, speed=120):
        self._send("B", speed)

    def left(self):
        self._send("L", 0)

    def right(self):
        self._send("R", 0)

    def stop(self):
        self._send("S", 0)

    # Servo control
    def servo(self, angle):
        angle = max(0, min(180, angle))
        self.serial.send(f"SERVO {angle}")

    # Execute arbitrary command (Navigator uses this)
    def execute(self, command, speed=120):
        if command not in self.valid_commands:
            print(f"[MotionController] Invalid command: {command}")
            return

        self._send(command, speed)

    # Internal send helper
    def _send(self, command, speed):
        # Sends to ESP32
        self.serial.send(command)

        # Notify localization (if provided)
        if self.localization:
            self.localization.notify_motion(command, speed)



