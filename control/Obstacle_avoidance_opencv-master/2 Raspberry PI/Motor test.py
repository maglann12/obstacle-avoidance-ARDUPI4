import serial
import time

# Initialize serial communication with Arduino
arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
time.sleep(2)

def send_command(command):
    arduino.write(command.encode())

def forward():
    send_command('F')

def backward():
    send_command('B')

def right():
    send_command('R')

def left():
    send_command('L')

def stop():
    send_command('S')

# Test motor control
forward()
time.sleep(2)
right()
time.sleep(2)
left()
time.sleep(2)
stop()