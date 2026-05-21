import serial

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

def read_ultrasonic():
    line = ser.readline().decode().strip()
    if line.startswith("ULTRASONIC:"):
       return int(line.split(":")[1])
    return None

def send(command):
    ser.write((command + "\n").encode())