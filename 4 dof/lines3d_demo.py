from tkinter import *
import sys
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
import time
import math

temp = 0
last = 0
angle = np.linspace(0, 359, num=360)
fig1 = plt.figure(1)
fig2 = plt.figure(2)
ar = fig1.gca(projection='3d')
sf = fig2.add_subplot(111)
sf.grid(True)
ar.set_xlim(-50, 50)
ar.set_ylim(-50, 50)
ar.set_zlim(-50, 50)
t_1 = 0
t_2 = 0
t_3 = 0
t_4 = 0
t_5 = 0
t_6 = 0
st_x = 0
st_y = 0
st_z = 10
r1 = 10
r2 = 20
r3 = 10


def to_RAD(deg):
    return float(deg)*(math.pi / 180)


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
    r_1 = np.array([r_x_1, r_y_1, r_z_1])

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


x_3, y_3, z_3 = end_point(t_1, t_2, t_3, t_4, t_5, 90)
xd = 1
the = 0
while 1:
    print(the)
    st_x = 30*math.cos(to_RAD(the))
    st_y = 30*math.sin(to_RAD(the))
    st_z = the/10
    c_f_1 = []
    c_f_2 = []
    c_f_3 = []
    c_f_4 = []
    c_f_5 = []
    c_f_6 = []
    cost_func = cost(x_3, y_3, z_3, st_x, st_y, st_z)
    print('cost_function:', cost_func)
    if cost_func < 2:
        the += 1
    for counter in angle:
        i, j, k = end_point(counter, t_2, t_3, t_4, t_5, t_6)
        cost_func = cost(i, j, k, st_x, st_y, st_z)
        c_f_1.append(cost_func)
    for counter in angle:
        i, j, k = end_point(t_1, counter, t_3, t_4, t_5, t_6)
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
        i, j, k = end_point(t_1, t_2, t_3, t_4, counter, t_6)
        cost_func = cost(i, j, k, st_x, st_y, st_z)
        c_f_5.append(cost_func)
    for counter in angle:
        i, j, k = end_point(t_1, t_2, t_3, t_4, t_5, counter)
        cost_func = cost(i, j, k, st_x, st_y, st_z)
        c_f_6.append(cost_func)
    if xd == 1:

        t_5 = c_f_5.index(min(c_f_5))
        #print('t_1:',t_1)
    if xd == 2:
        t_6 = c_f_6.index(min(c_f_6))
        #print('t_2:',t_2)

    if xd == 3:
        t_1 = c_f_1.index(min(c_f_1))
        #print('t_3:',t_3)

    if xd == 4:
        t_2 = c_f_2.index(min(c_f_2))
        #print('t_4:',t_4)

    if xd == 5:
        t_3 = c_f_3.index(min(c_f_3))
        #print('t_5:',t_5)

    if xd == 6:
        t_4 = c_f_4.index(min(c_f_4))
        #print('t_6:',t_6)
        xd = 0




    #print(t_1, t_2, t_3, t_4, t_5, t_6)
    #arm move
    x_3, y_3, z_3, r_x_1, r_y_1, r_z_1, r_x_2, r_y_2, r_z_2, r_x_3, r_y_3, r_z_3 = forward_kinematics(t_1, t_2, t_3, t_4, t_5, t_6)
    #print(t_1, t_2, t_3, t_4, t_5, t_6)

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

    ar.set_xlabel('X - axis')
    ar.set_ylabel('Y - axis')
    ar.set_zlabel('Z - axis')

    xd += 1


    plt.pause(0.01)
    line1.remove()
    line2.remove()
    line3.remove()
    #tgt.remove()
    angle1.remove()
    angle2.remove()
    angle3.remove()
    angle4.remove()
    angle5.remove()
    angle6.remove()