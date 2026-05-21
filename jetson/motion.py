"""
Responsibilities:

High-level movement commands sent to the ESP32.
This module abstracts away the serial commands.
"""
def MotionController(serial):
    print("MotionController: Not implemented yet")

def forward(speed):
    return None 
#Will return a string command to send to the ESP32 (ex: "FORWARD:100")

def backward(speed):
    #Will do later
    #Will return a string command to send to the ESP32 (ex: "BACKWARD:100")
    return None

def turn_left(speed):
    #Will do later
    #Will return a string command to send to the ESP32 (ex: "TURN_LEFT:100")
    return None

def turn_right(speed):
    #Will do later
    #Will return a string command to send to the ESP32 (ex: "TURN_RIGHT:100")
    return None

def stop():
    #Will do later
    #Will return a string command to send to the ESP32 (ex: "STOP")
    return None

def servo(angle):
    #Will do later
    #Will return a string command to send to the ESP32 (ex: "SERVO:90")
    return None


