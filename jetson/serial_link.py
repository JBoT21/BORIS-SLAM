import serial

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

def read_ultrasonic():
    line = ser.readline().decode().strip()
    if line.startswith("ULTRASONIC:"):
       return int(line.split(":")[1])
    return None

#def read_line():


#def read_imu():
    #IMU is used to determine robot orientation


#def send(command):
#    ser.write((command + "\n").encode())