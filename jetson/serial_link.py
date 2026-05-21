import serial

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

def read_ultrasonic():
    line = ser.readline().decode().strip()
    if line.startswith("ULTRASONIC:"):
       return int(line.split(":")[1])
    return None

def read_line():
    print("read_line: Not implemented yet")

def read_imu():
    print("read_imu: Not implemented yet")


def send(command):
    print("send: Not implemented fully yet")
    
    ser.write((command + "\n").encode())
    print(f"Sent command: {command}")

def serialLink():
    print("SerialLink: Not implemented yet")    