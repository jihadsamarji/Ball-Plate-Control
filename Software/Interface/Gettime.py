import cv2
import numpy as np
import time
import imutils
import tkinter as tk
import tkinter.messagebox
from PIL import Image, ImageTk
import serial
from datetime import datetime
import serial.tools.list_ports
from math import *

arduinoIsConnected = False

def connectArduino():   # function that checks if arduino is connected or not, initialize the serial and toggles the bools
    global ser
    global arduinoIsConnected
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if "Arduino" or "CH340" in p.description:
            print(p)
            ser = serial.Serial(p[0], 19200, timeout=1)
            time.sleep(1)  # give the connection a second to settle
            arduinoIsConnected = True

connectArduino()
time.sleep(10)
if arduinoIsConnected == True:
    print("arduino connected")
    print(datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3])
    ser.write((str(9) + "," + str(5) + "\n").encode())
    ser.flush()
    print(datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3])
