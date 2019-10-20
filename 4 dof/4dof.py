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

try:
    port = 'COM8'  # "/dev/tty.HC-06-DevB"
    print("Connecting to device connected to " + port)
    arduino = serial.Serial(port, 9600, timeout=.1)
    print("Connected")
except:
    print("Not connected")
    pass
time.sleep(5)
temp = 0
last = 0

fig = plt.figure(1)
ar = fig.gca(projection='3d')


ar.set_xlim(-50, 50)
ar.set_ylim(-50, 50)
ar.set_zlim(-50, 50)
sd1 = 0
sd2 = 0
sd3 = 0
sd4 = 0

st_x = 0
st_y = 0
st_z = 0
r1 = 28
r2 = 18
r3 = 14
R = r1 + r2 + r3



tk = Tk()
def constrain(amt,low,high):
    if amt < low:
        amt = low
    elif amt > high:
        amt = high
    return amt
def to_RAD(deg):
    return float(deg)*(math.pi / 180)


def slider1(val):
    global sd1
    sd1 = int(val)


def slider2(val):
    global sd2
    sd2 = int(val)


def slider3(val):
    global sd3
    sd3 = int(val)


def slider4(val):
    global sd4
    sd4 = int(val)


def sliderx(val):
    global st_x
    st_x = int(val)


def slidery(val):
    global st_y
    st_y = int(val)


def sliderz(val):
    global st_z
    st_z = int(val)



slider_x = Scale(tk, from_=-50, to=40, orient=HORIZONTAL, length=400, tickinterval=25, command=sliderx, label='Set_point_x')
slider_x.pack()
slider_y = Scale(tk, from_=-50, to=50, orient=HORIZONTAL, length=400, tickinterval=25, command=slidery, label='Set_point_y')
slider_y.pack()
slider_z = Scale(tk, from_=-50, to=50, orient=HORIZONTAL, length=400, tickinterval=25, command=sliderz, label='Set_point_z')
slider_z.pack()

slider_x.set(0)
slider_y.set(40)
slider_z.set(40)
def end_point(d1, d2, d4, d6):
    x_0 = 0
    y_0 = 0
    z_0 = 0

    x_1 = r1 * math.cos(to_RAD(d1)) * math.cos(to_RAD(d2)) + x_0
    y_1 = r1 * math.sin(to_RAD(d1)) * math.cos(to_RAD(d2)) + y_0
    z_1 = r1 * math.sin(to_RAD(d2)) + z_0

    r_x_1 = np.array([x_0, x_1])
    r_y_1 = np.array([y_0, y_1])
    r_z_1 = np.array([x_0, z_1])
    r_1 = np.array([r_x_1, r_y_1, r_z_1])

    x_2 = (r2 * math.cos(to_RAD(d1)) * math.cos(to_RAD(d2 + d4))) + x_1
    y_2 = (r2 * math.sin(to_RAD(d1)) * math.cos(to_RAD(d4 + d2))) + y_1
    z_2 = (r2 * math.sin(to_RAD(d4 + d2))) + z_1

    r_x_2 = np.array([x_1, x_2])
    r_y_2 = np.array([y_1, y_2])
    r_z_2 = np.array([z_1, z_2])

    x_3 = (r3 * math.cos(to_RAD(d1)) * math.cos(to_RAD(d2 + d4 + d6))) + x_2
    y_3 = (r3 * math.sin(to_RAD(d1)) * math.cos(to_RAD(d4 + d2 + d6))) + y_2
    z_3 = (r3 * math.sin(to_RAD(d4 + d2 + d6))) + z_2

    r_x_3 = np.array([x_2, x_3])
    r_y_3 = np.array([y_2, y_3])
    r_z_3 = np.array([z_2, z_3])
    return x_3, y_3, z_3



def forward_kinematics(d1, d2, d4, d6):
    x_0 = 0
    y_0 = 0
    z_0 = 0

    x_1 = r1 * math.cos(to_RAD(d1)) * math.cos(to_RAD(d2)) + x_0
    y_1 = r1 * math.sin(to_RAD(d1)) * math.cos(to_RAD(d2)) + y_0
    z_1 = r1 * math.sin(to_RAD(d2)) + z_0

    r_x_1 = np.array([x_0, x_1])
    r_y_1 = np.array([y_0, y_1])
    r_z_1 = np.array([x_0, z_1])
    r_1 = np.array([r_x_1, r_y_1, r_z_1])

    x_2 = (r2 * math.cos(to_RAD(d1)) * math.cos(to_RAD(d2 + d4))) + x_1
    y_2 = (r2 * math.sin(to_RAD(d1)) * math.cos(to_RAD(d4 + d2))) + y_1
    z_2 = (r2 * math.sin(to_RAD(d4 + d2))) + z_1

    r_x_2 = np.array([x_1, x_2])
    r_y_2 = np.array([y_1, y_2])
    r_z_2 = np.array([z_1, z_2])

    x_3 = (r3 * math.cos(to_RAD(d1)) * math.cos(to_RAD(d2 + d4 + d6))) + x_2
    y_3 = (r3 * math.sin(to_RAD(d1)) * math.cos(to_RAD(d4 + d2 + d6))) + y_2
    z_3 = (r3 * math.sin(to_RAD(d4 + d2 + d6))) + z_2

    r_x_3 = np.array([x_2, x_3])
    r_y_3 = np.array([y_2, y_3])
    r_z_3 = np.array([z_2, z_3])
    return x_3, y_3, z_3, r_x_1, r_y_1, r_z_1, r_x_2, r_y_2, r_z_2, r_x_3, r_y_3, r_z_3


def inverse_kinematics(x, y, z):
    """
double base_angle, arm_angle, elbow_angle,wrist_angle;
double  r1 = 28, r2 = 18, r3 = 5;//arm elbow wrist
double R = r1+r2+r3;
if(x!=0)
base_angle=atan2(y,x) + PI/2;


double r = sqrt((x * x) + (y * y));
double d = sqrt(((r - r3) * (r - r3)) + (z * z));
double theta = atan2(z, r - r3);
double A = ((r1 * r1)+ (d * d) - (r2 * r2)) / (2 * r1 * d);
A = constrain(A, -1, 1);
A = acos(A);
arm_angle = theta + A;
double B = ((r1 * r1)+ (r2 * r2) - (d * d)) / (2 * r1 * r2);
B = constrain(B, -1, 1);
B = acos(B);
elbow_angle = PI - B;
double C = ((r2 * r2)+ (d * d) - (r1 * r1)) / (2 * r2 * d);
C = constrain(C, -1, 1);
C = acos(C);
wrist_angle = C - (theta) + PI/2;
"""
    a1 = math.atan2(y, x)


    r = math.sqrt((x * x) + (y * y))

    d = math.sqrt(((r - r3) * (r - r3)) + (z * z))

    theta = math.atan2(z, r - r3)

    A = ((r1 * r1) + (d * d) - (r2 * r2)) / (2 * r1 * d)
    A = constrain(A, -1, 1)
    A = math.acos(A)
    a2 = theta + A

    B = ((r1 * r1) + (r2 * r2) - (d * d)) / (2 * r1 * r2)
    B = constrain(B, -1, 1)
    B = math.acos(B)
    a3 = B - math.pi
    C = ((r2 * r2) + (d * d) - (r1 * r1)) / (2 * r2 * d)
    C = constrain(C, -1, 1)
    C = math.acos(C)
    a4 = C - theta
    #print(math.degrees(a1), math.degrees(a2), math.degrees(a3), math.degrees(a4))
    return math.degrees(a1), math.degrees(a2), math.degrees(a3), math.degrees(a4)




while 1:

#    x_3, y_3, z_3, r_x_1, r_y_1, r_z_1, r_x_2, r_y_2, r_z_2, r_x_3, r_y_3, r_z_3 = forward_kinematics(sd1, sd2, sd3, sd4)
    a11, a22, a33, a44 = inverse_kinematics(st_x, st_y, st_z)
    a11 = float("{0:.3f}".format(a11))
    a22 = float("{0:.3f}".format(a22))
    a33 = float("{0:.3f}".format(a33))
    a44 = float("{0:.3f}".format(a44))

# print(t_1, t_2, t_3)
    data = str(float(a11)) + ',' + str(float(a22)) + ',' + str(float(a33)) + ',' + str(float(a44 - 90)) + '\n'
    print(data)
    data = data.encode('utf-8')
    #print(data)
    try:
        arduino.write(data)
    except:
        print("No Connection")
    x_3, y_3, z_3, r_x_1, r_y_1, r_z_1, r_x_2, r_y_2, r_z_2, r_x_3, r_y_3, r_z_3 = forward_kinematics(a11, a22, a33, a44)

# print('x_1, y_1, z_1:', x_1, y_1, z_1)
    # print('x_2, y_2, z_2:', x_2, y_2, z_2)
    # print('x_3, y_3, z_3:', x_3, y_3, z_3)

    line1, = ar.plot(r_x_1, r_y_1, r_z_1, 'r-*')
    line2, = ar.plot(r_x_2, r_y_2, r_z_2, 'g-*')
    line3, = ar.plot(r_x_3, r_y_3, r_z_3, 'b-*')
    tgt = ar.scatter(st_x, st_y, st_z, color='black')

    ar.set_xlabel('X - axis')
    ar.set_ylabel('Y - axis')
    ar.set_zlabel('Z - axis')

    tk.update_idletasks()
    tk.update()
    plt.pause(0.1)

    line1.remove()
    line2.remove()
    line3.remove()
    tgt.remove()






