from tkinter import *
import sys
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
import time
import math
import operator

temp = 0
last = 0
angle = np.linspace(0, 359, num=360)
#fig1 = plt.figure(1)
#fig2 = plt.figure(2)
#fig3 = plt.figure(3)
fig4 = plt.figure(4)
#fig5 = plt.figure(5)
fig6 = plt.figure(6)
ar = fig4.gca(projection='3d')
#ct = fig5.add_subplot(111)
sf = fig6.add_subplot(111)

#xy = fig1.add_subplot(111)
#yz = fig2.add_subplot(111)
#xz = fig3.add_subplot(111)

ar.set_xlim(-50, 50)
ar.set_ylim(-50, 50)
ar.set_zlim(-50, 50)
sd1 = 0
sd2 = 0
sd3 = 0
sd4 = 0
sd5 = 0
sd6 = 0
st_x = 0
st_y = 0
st_z = 0
r1 = 10
r2 = 20
r3 = 10
tk = Tk()




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
    sd3 = 0#int(val)


def slider4(val):
    global sd4
    sd4 = int(val)


def slider5(val):
    global sd5
    sd5 = 0#int(val)


def slider6(val):
    global sd6
    sd6 = int(val)

def sliderx(val):
    global st_x
    st_x = int(val)


def slidery(val):
    global st_y
    st_y = int(val)


def sliderz(val):
    global st_z
    st_z = int(val)


slider1 = Scale(tk, from_=0, to=360, orient=HORIZONTAL, length=400, tickinterval=25, command=slider1, label='Theta_1')
slider1.pack()
slider2 = Scale(tk, from_=0, to=360, orient=HORIZONTAL, length=400, tickinterval=25, command=slider2, label='Theta_2')
slider2.pack()
#slider3 = Scale(tk, from_=0, to=360, orient=HORIZONTAL, length=400, tickinterval=25, command=slider3, label='Theta_3')
#slider3.pack()
slider4 = Scale(tk, from_=0, to=360, orient=HORIZONTAL, length=400, tickinterval=25, command=slider4, label='Theta_4')
slider4.pack()
#slider5 = Scale(tk, from_=0, to=360, orient=HORIZONTAL, length=400, tickinterval=25, command=slider5, label='Theta_5')
#slider5.pack()
slider6 = Scale(tk, from_=0, to=360, orient=HORIZONTAL, length=400, tickinterval=25, command=slider6, label='Theta_6')
slider6.pack()
slider_x = Scale(tk, from_=-40, to=40, orient=HORIZONTAL, length=400, tickinterval=25, command=sliderx, label='Set_point_x')
slider_x.pack()
slider_y = Scale(tk, from_=-40, to=40, orient=HORIZONTAL, length=400, tickinterval=25, command=slidery, label='Set_point_y')
slider_y.pack()
slider_z = Scale(tk, from_=-40, to=40, orient=HORIZONTAL, length=400, tickinterval=25, command=sliderz, label='Set_point_z')
slider_z.pack()


def end_point(d1, d2, d3, d4, d5, d6):
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
while 1:
    x_3, y_3, z_3, r_x_1, r_y_1, r_z_1, r_x_2, r_y_2, r_z_2, r_x_3, r_y_3, r_z_3 = forward_kinematics(sd1, sd2, sd3,
                                                                                                      sd4, sd5, sd6)
    # print('x_1, y_1, z_1:', x_1, y_1, z_1)
    # print('x_2, y_2, z_2:', x_2, y_2, z_2)
    # print('x_3, y_3, z_3:', x_3, y_3, z_3)

    line1, = ar.plot(r_x_1, r_y_1, r_z_1, 'r-*')
    line2, = ar.plot(r_x_2, r_y_2, r_z_2, 'g-*')
    line3, = ar.plot(r_x_3, r_y_3, r_z_3, 'b-*')
    tgt = ar.scatter(st_x, st_y, st_z, color='black')

    #    XY1, = xy.plot(r_x_1, r_y_1, 'r-*')
    #    XY2, = xy.plot(r_x_2, r_y_2, 'g-*')
    #    XY3, = xy.plot(r_x_3, r_y_3, 'b-*')

    #    YZ1, = yz.plot(r_y_1, r_z_1, 'r-*')
    #    YZ2, = yz.plot(r_y_2, r_z_2, 'g-*')
    #    YZ3, = yz.plot(r_y_3, r_z_3, 'b-*')

    #    XZ1, = xz.plot(r_x_1, r_z_1, 'r-*')
    #    XZ2, = xz.plot(r_x_2, r_z_2, 'g-*')
    #    XZ3, = xz.plot(r_x_3, r_z_3, 'b-*')

    ar.set_xlabel('X - axis')
    ar.set_ylabel('Y - axis')
    ar.set_zlabel('Z - axis')

    #    xy.set_xlabel('X - axis')
    #    xy.set_ylabel('Y - axis')

    #    yz.set_xlabel('Y - axis')
    #    yz.set_ylabel('Z - axis')

    #    xz.set_xlabel('X - axis')
    #    xz.set_ylabel('Z - axis')



    #    XY1.remove()
    #    XY2.remove()
    #    XY3.remove()
    #    YZ1.remove()
    #    YZ2.remove()
    #    YZ3.remove()
    #    XZ1.remove()
    #    XZ2.remove()
    #    XZ3.remove()




    cost_func = cost(x_3, y_3, z_3, st_x, st_y, st_z)
    #curve1, = ct.plot(sd1, cost_func, 'r-*')
    #curve2, = ct.plot(sd2, cost_func, 'g-*')
    #curve3, = ct.plot(sd3, cost_func, 'b-*')
    #curve4, = ct.plot(sd4, cost_func, 'c-*')
    #curve5, = ct.plot(sd5, cost_func, 'm-*')
    #curve6, = ct.plot(sd6, cost_func, 'y-*')

    #print(cost_func)
    #ct.set_xlabel('Angle - axis')
    #ct.set_ylabel('cost_function - axis')

        #plt.pause(0.01)
        #curve1.remove()
        #curve2.remove()
        #curve3.remove()
        #curve4.remove()
        #curve5.remove()
        #curve6.remove()
    c_f_1 = []
    c_f_2 = []
    c_f_3 = []
    c_f_4 = []
    c_f_5 = []
    c_f_6 = []

    for counter in angle:
        i, j, k = end_point(counter, sd2, sd3, sd4, sd5, sd6)
        cost_func = cost(i, j, k, st_x, st_y, st_z)
        c_f_1 .append(cost_func)
    angle1, = sf.plot(angle, c_f_1, 'r')
    for counter in angle:
        i, j, k = end_point(sd1, counter, sd3, sd4, sd5, sd6)
        cost_func = cost(i, j, k, st_x, st_y, st_z)
        c_f_2.append(cost_func)
    angle2, = sf.plot(angle, c_f_2, 'g')
    for counter in angle:
        i, j, k = end_point(sd1, sd2, counter, sd4, sd5, sd6)
        cost_func = cost(i, j, k, st_x, st_y, st_z)
        c_f_3.append(cost_func)
    angle3, = sf.plot(angle, c_f_3, 'b')
    for counter in angle:
        i, j, k = end_point(sd1, sd2, sd3, counter, sd5, sd6)
        cost_func = cost(i, j, k, st_x, st_y, st_z)
        c_f_4.append(cost_func)
    angle4, = sf.plot(angle, c_f_4, 'c')
    for counter in angle:
        i, j, k = end_point(sd1, sd2, sd3, sd4, counter, sd6)
        cost_func = cost(i, j, k, st_x, st_y, st_z)
        c_f_5.append(cost_func)
    angle5, = sf.plot(angle, c_f_5, 'm')
    for counter in angle:
        i, j, k = end_point(sd1, sd2, sd3, sd4, sd5, counter)
        cost_func = cost(i, j, k, st_x, st_y, st_z)
        c_f_6.append(cost_func)
    angle6, = sf.plot(angle, c_f_6, 'k')
    #minposition = A.index(min(A))
    t_1 = c_f_1.index(min(c_f_1))
    t_2 = c_f_2.index(min(c_f_2))
    t_3 = c_f_3.index(min(c_f_3))
    t_4 = c_f_4.index(min(c_f_4))
    t_5 = c_f_5.index(min(c_f_5))
    t_6 = c_f_6.index(min(c_f_6))
    print(t_1, t_2, t_3, t_4, t_5, t_6)
#    min_index, min_value = min(c_f_6, key=operator.itemgetter(1))
#    print(min_value)
    tk.update_idletasks()
    tk.update()
    plt.pause(0.1)

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





