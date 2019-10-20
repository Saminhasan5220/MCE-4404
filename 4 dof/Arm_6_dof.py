from tkinter import *
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
import math
import serial

#port = 'COM23'  # "/dev/tty.HC-06-DevB"
#print("Connecting to device connected to " + port)
#bluetooth = serial.Serial(port, 9600, timeout=.1)
#print("Connected")

angle = np.linspace(0, 360, num=361)
fig1 = plt.figure(1)
fig2 = plt.figure(2)
ar = fig1.gca(projection='3d')
sf = fig2.add_subplot(111)
sf.grid(True)
ar.set_xlim(-50, 50)
ar.set_ylim(-50, 50)
ar.set_zlim(-50, 50)
ar.set_xlabel('X - axis')
ar.set_ylabel('Y - axis')
ar.set_zlabel('Z - axis')

t_1 = 0
t_2 = 0
t_3 = 0
t_4 = 0
t_5 = 0
t_6 = 0
st_x = 0
st_y = 0
st_z = 0
r1 = 10
r2 = 20
r3 = 10
tk = Tk()


def send(a, b, c, d, e, f):
    print((str(a) + ',' + str(b) + ',' + str(c) + ',' + str(d) + ',' + str(e) + ',' + str(f) + '\n'))
    bluetooth.write(str.encode((str(a) + ',' + str(b) + ',' + str(c) + ',' + str(d) + ',' + str(e) + ',' + str(f) + '\n')))


def to_RAD(deg):
    return float(deg)*(math.pi / 180)

def sliderx(val):
    global st_x
    st_x = int(val)


def slidery(val):
    global st_y
    st_y = int(val)


def sliderz(val):
    global st_z
    st_z = int(val)


slider_x = Scale(tk, from_=-40, to=40, orient=HORIZONTAL, length=400, tickinterval=25, command=sliderx, label='Setpoint_x')
slider_x.pack()
slider_y = Scale(tk, from_=-40, to=40, orient=HORIZONTAL, length=400, tickinterval=25, command=slidery, label='Setpoint_y')
slider_y.pack()
slider_z = Scale(tk, from_=-40, to=40, orient=HORIZONTAL, length=400, tickinterval=25, command=sliderz, label='Setpoint_z')
slider_z.pack()


def end_point(d1, d2, d3, d4, d5, d6):
    x_0 = 0
    y_0 = 0
    z_0 = 0

    x_1 = r1 * math.cos(to_RAD(d1)) * math.cos(to_RAD(d2)) + x_0
    y_1 = r1 * math.sin(to_RAD(d1)) * math.cos(to_RAD(d2)) + y_0
    z_1 = r1 * math.sin(to_RAD(d2)) + z_0

    x_2 = (r2 * math.cos(to_RAD(d3 + d1)) * math.cos(to_RAD(d2 + d4))) + x_1
    y_2 = (r2 * math.sin(to_RAD(d3 + d1)) * math.cos(to_RAD(d4 + d2))) + y_1
    z_2 = (r2 * math.sin(to_RAD(d4 + d2))) + z_1

    x_3 = (r3 * math.cos(to_RAD(d3 + d1 + d5)) * math.cos(to_RAD(d2 + d4 + d6))) + x_2
    y_3 = (r3 * math.sin(to_RAD(d3 + d1 + d5)) * math.cos(to_RAD(d4 + d2 + d6))) + y_2
    z_3 = (r3 * math.sin(to_RAD(d4 + d2 + d6))) + z_2

    return x_3, y_3, z_3


def forward_kinematics(d1, d2, d3, d4, d5, d6):
    x_0 = 0
    y_0 = 0
    z_0 = 0

    x_1 = r1 * math.cos(to_RAD(d1)) * math.cos(to_RAD(d2)) + x_0
    y_1 = r1 * math.sin(to_RAD(d1)) * math.cos(to_RAD(d2)) + y_0
    z_1 = r1 * math.sin(to_RAD(d2)) + z_0

    r_x_1 = np.array([x_0, x_1])
    r_y_1 = np.array([y_0, y_1])
    r_z_1 = np.array([x_0, z_1])

    x_2 = (r2 * math.cos(to_RAD(d3 + d1)) * math.cos(to_RAD(d2 + d4))) + x_1
    y_2 = (r2 * math.sin(to_RAD(d3 + d1)) * math.cos(to_RAD(d4 + d2))) + y_1
    z_2 = (r2 * math.sin(to_RAD(d4 + d2))) + z_1

    r_x_2 = np.array([x_1, x_2])
    r_y_2 = np.array([y_1, y_2])
    r_z_2 = np.array([z_1, z_2])

    x_3 = (r3 * math.cos(to_RAD(d3 + d1 + d5)) * math.cos(to_RAD(d2 + d4 + d6))) + x_2
    y_3 = (r3 * math.sin(to_RAD(d3 + d1 + d5)) * math.cos(to_RAD(d4 + d2 + d6))) + y_2
    z_3 = (r3 * math.sin(to_RAD(d4 + d2 + d6))) + z_2

    r_x_3 = np.array([x_2, x_3])
    r_y_3 = np.array([y_2, y_3])
    r_z_3 = np.array([z_2, z_3])
    return x_3, y_3, z_3, r_x_1, r_y_1, r_z_1, r_x_2, r_y_2, r_z_2, r_x_3, r_y_3, r_z_3


def cost(x_c, y_c, z_c, t_x, t_y, t_z):
    cost_function = math.sqrt((x_c - t_x) ** 2 + (y_c - t_y) ** 2 + (z_c - t_z) ** 2)
    return float(cost_function)


x_3, y_3, z_3 = end_point(t_1, t_2, t_3, t_4, t_5, t_6)
cnt = 1

while 1:

    c_f_1 = []
    c_f_2 = []
    c_f_3 = []
    c_f_4 = []
    c_f_5 = []
    c_f_6 = []
    cost_func = cost(x_3, y_3, z_3, st_x, st_y, st_z)
    distance = math.sqrt(x_3**2 + y_3**2 + z_3**2)
    #print('dis:', distance)
    print('cost_function:', cost_func)
    for counter in angle:
        i, j, k = end_point(counter, t_2, t_3, t_4, t_5, t_6)
        cost_func = cost(i, j, k, st_x, st_y, st_z)
        c_f_1.append(cost_func)
    for counter in angle:
        i, j, k = end_point(t_1, counter,  t_3, t_4, t_5, t_6)
        cost_func = cost(i, j, k, st_x, st_y, st_z)
        c_f_2.append(cost_func)
    for counter in angle:
        i, j, k = end_point(t_1, t_2, counter, t_4, t_5, t_6)
        cost_func = cost(i, j, k, st_x, st_y, st_z)
        c_f_3.append(cost_func)
    for counter in angle:
        i, j, k = end_point(t_1, t_2, t_3, counter, t_5, t_6)
        cost_func = cost(i, j, k, st_x, st_y, st_z)
        c_f_4.append(cost_func)
    for counter in angle:
        i, j, k = end_point(t_1, t_2, t_3, t_4, counter, 0)
        cost_func = cost(i, j, k, st_x, st_y, st_z)
        c_f_5.append(cost_func)
    for counter in angle:
        i, j, k = end_point(t_1, t_2, t_3, t_4, t_5, counter)
        cost_func = cost(i, j, k, st_x, st_y, st_z)
        c_f_6.append(cost_func)
    if cnt == 1:
        t_5 = c_f_5.index(min(c_f_5))
    if cnt == 2:
        t_6 = c_f_6.index(min(c_f_6))

    if cnt == 3:
        c_f_1.index(min(c_f_1))

    if cnt == 4:
        c_f_2.index(min(c_f_2))

    if cnt == 5:
        t_3 = c_f_3.index(min(c_f_3))

    if cnt == 6:
        t_4 = c_f_4.index(min(c_f_4))
        cnt = 0
    cnt += 1

    #print( t_1, t_2, t_3, t_4, t_5, t_6)
    #arm move
    x_3, y_3, z_3, r_x_1, r_y_1, r_z_1, r_x_2, r_y_2, r_z_2, r_x_3, r_y_3, r_z_3 = forward_kinematics(t_1, t_2, t_3, t_4, t_5, t_6)
    #print(t_1, t_2, t_3, t_4, t_5, t_6)
    #send(t_1, t_2, t_3, t_4, t_5, t_6)

    line1, = ar.plot(r_x_1, r_y_1, r_z_1, 'r-*')
    line2, = ar.plot(r_x_2, r_y_2, r_z_2, 'g-*')
    line3, = ar.plot(r_x_3, r_y_3, r_z_3, 'b-*')
    tgt = ar.scatter(st_x, st_y, st_z, color='black')

    angle1, = sf.plot(angle, c_f_1, 'r')
    angle2, = sf.plot(angle, c_f_2, 'g')
    angle3, = sf.plot(angle, c_f_3, 'b')
    angle4, = sf.plot(angle, c_f_4, 'c')
    angle5, = sf.plot(angle, c_f_5, 'm')
    angle6, = sf.plot(angle, c_f_6, 'k')


    tk.update_idletasks()
    tk.update()
    plt.pause(0.00000001)
    line1.remove()
    line2.remove()
    line3.remove()
    tgt.remove()
    angle1.remove()
    angle2.remove()
    angle3.remove()
    angle4.remove()
    angle5.remove()
    angle6.remove()






