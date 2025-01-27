import cv2
import numpy as np
import math
import os
import serial
from time import sleep

# Initialize serial communication with Arduino
arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  # Adjust port and baud rate
sleep(2)  # Wait for the connection to establish

def send_command(command):
    arduino.write(command.encode())  # Send command to Arduino

def forward():
    send_command('F')  # Send 'F' to move forward

def backward():
    send_command('B')  # Send 'B' to move backward

def right():
    send_command('R')  # Send 'R' to turn right

def left():
    send_command('L')  # Send 'L' to turn left

def stop():
    send_command('S')  # Send 'S' to stop

def calc_dist(p1, p2):
    x1 = p1[0]
    y1 = p1[1]
    x2 = p2[0]
    y2 = p2[1]
    dist = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return dist

def getChunks(l, n):
    """Yield successive n-sized chunks from l."""
    a = []
    for i in range(0, len(l), n):
        a.append(l[i:i + n])
    return a

cap = cv2.VideoCapture(0)
StepSize = 5
currentFrame = 0

try:
    if not os.path.exists('data'):
        os.makedirs('data')
except OSError:
    print('Error: Creating directory of data')

if testmode == 1:
    F = open("./data/imagedetails.txt", 'a')
    F.write("\n\nNew Test \n")

while True:
    _, frame = cap.read()
    img = frame.copy()
    blur = cv2.bilateralFilter(img, 9, 40, 40)
    edges = cv2.Canny(blur, 50, 100)
    img_h = img.shape[0] - 1
    img_w = img.shape[1] - 1
    EdgeArray = []

    for j in range(0, img_w, StepSize):
        pixel = (j, 0)
        for i in range(img_h - 5, 0, -1):
            if edges.item(i, j) == 255:
                pixel = (j, i)
                break
        EdgeArray.append(pixel)

    chunks = getChunks(EdgeArray, int(len(EdgeArray) / 3))
    c = []
    for i in range(len(chunks) - 1):
        x_vals = []
        y_vals = []
        for (x, y) in chunks[i]:
            x_vals.append(x)
            y_vals.append(y)
        avg_x = int(np.average(x_vals))
        avg_y = int(np.average(y_vals))
        c.append([avg_y, avg_x])
        cv2.line(frame, (320, 480), (avg_x, avg_y), (255, 0, 0), 2)

    forwardEdge = c[1]
    cv2.line(frame, (320, 480), (forwardEdge[1], forwardEdge[0]), (0, 255, 0), 3)

    if forwardEdge[0] > 250:  # Obstacle detected
        if y[1] < 310:
            left()  # Turn left
        else:
            right()  # Turn right
    else:
        forward()  # Move forward

    if testmode == 1:
        F.write("frame" + str(currentFrame) + ".jpg" + " | " + str(c[0]) + " | " + str(c[1]) + " | " + str(c[2]) + " | " + direction + "\n")
        currentFrame += 1

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cap.release()
cv2.destroyAllWindows()