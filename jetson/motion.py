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

    def forward(self):
        self.serial.send("F")

    def backward(self):
        self.serial.send("B")

    def left(self):
        self.serial.send("L")

    def right(self):
        self.serial.send("R")

    def stop(self):
        self.serial.send("S")

    def servo(self, angle):
        angle = max(0, min(180, angle))
        self.serial.send(f"SERVO {angle}")

    def execute(self, command):
        self.serial.send(command)



