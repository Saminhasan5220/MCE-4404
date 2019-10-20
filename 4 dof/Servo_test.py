from tkinter import *
import sys
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
import time
import math
import operator
import serial
sd1 = 0
try:
    port = 'COM3'  # "/dev/tty.HC-06-DevB"
    print("Connecting to device connected to " + port)
    bluetooth = serial.Serial(port, 9600, timeout=.1)
    print("Connected")
except:
    print("Not connected")
    pass
time.sleep(5)


st_x = 0
tk = Tk()


def slider1(val):
    global sd1
    sd1 = int(val)


slider1 = Scale(tk, from_=0, to=180, orient=HORIZONTAL, length=400, tickinterval=25, command=slider1, label='Theta_1')
slider1.pack()

while 1:
    time.sleep(0.05)
    data = str(int(sd1))  + '\n'
    data = data.encode('utf-8')
    # print(data)
    bluetooth.write(data)
    tk.update_idletasks()
    tk.update()