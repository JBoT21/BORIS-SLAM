"""
Responsibilities:

High-level movement commands sent to the ESP32.
This module abstracts away the serial commands.
"""
"""
motion.py — High-level movement commands for ESP32 motor controller.
"""

class MotionController:

    def __init__(self, serial_link):
        self.serial = serial_link

    def forward(self, speed=150):
        cmd = f"MOVE FWD {speed}"
        self.serial.send(cmd)

    def backward(self, speed=150):
        cmd = f"MOVE BACK {speed}"
        self.serial.send(cmd)

    def turn_left(self, speed=150):
        cmd = f"TURN LEFT {speed}"
        self.serial.send(cmd)

    def turn_right(self, speed=150):
        cmd = f"TURN RIGHT {speed}"
        self.serial.send(cmd)

    def stop(self):
        self.serial.send("STOP")


    def servo(self, angle):
        angle = max(0, min(180, angle))  # clamp
        self.serial.send(f"SERVO {angle}")

    def execute(self, command):
        """Allows Navigator to send arbitrary commands."""
        self.serial.send(command)



