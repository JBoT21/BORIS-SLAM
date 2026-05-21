"""
Responsibilities:

High-level movement commands sent to the ESP32.
This module abstracts away the serial commands.
"""
def MotionController(serial):
    print("MotionController: Not implemented yet")

def forward(speed):
    return f"FORWARD:{speed}"

def backward(speed):
    return f"BACKWARD:{speed}"

def turn_left(speed):
    return f"TURN_LEFT:{speed}"

def turn_right(speed):
    return f"TURN_RIGHT:{speed}"

def stop():
    return "STOP"

def servo(angle):
   return f"SERVO:{angle}"

def execute(serial, command):
    serial.send(command)


