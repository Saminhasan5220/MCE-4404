
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
import cv2
import cv2.aruco as aruco
import sys, time, math
cap = cv2.VideoCapture(0)

#--- Define Tag
id_to_find  = 1
marker_size  = 3.7#- [cm]
xa,ya,za = 28,18,14

#######################################################################################################################
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


ar.set_xlim(-150, 150)
ar.set_ylim(-150, 150)
ar.set_zlim(-150, 150)

st_x = 0
st_y = 0
st_z = 0
r1 = 22.5
r2 = 17
r3 = 5
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

#######################################################################################################################
# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])




#--- Get the camera calibration path
calib_path  = ""
camera_matrix   = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')

#--- 180 deg rotation matrix around the x axis
R_flip  = np.zeros((3,3), dtype=np.float32)
R_flip[0,0] = 1.0
R_flip[1,1] =-1.0
R_flip[2,2] =-1.0

#--- Define the aruco dictionary
aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters  = aruco.DetectorParameters_create()


#--- Capture the videocamera (this may also be a video or a picture)
#-- Set the camera size as the one it was calibrated with
#cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

#-- Font for the text in the image
font = cv2.FONT_HERSHEY_PLAIN

while True:

    #-- Read the camera frame
    ret, frame = cap.read()

    #-- Convert in gray scale
    gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red

    #-- Find all the aruco markers in the image
    corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters,
                              cameraMatrix=camera_matrix, distCoeff=camera_distortion)

    if ids is not None:# and ids[0] == id_to_find:
        print('ID:' , ids)
        #-- ret = [rvec, tvec, ?]
        #-- array of rotation and position of each marker in camera frame
        #-- rvec = [[rvec_1], [rvec_2], ...]    attitude of the marker respect to camera frame
        #-- tvec = [[tvec_1], [tvec_2], ...]    position of the marker in camera frame
        for cr in corners:
            #print('IDS FOUND =' + str(len(corners)))
            #print(corners)
            ret = aruco.estimatePoseSingleMarkers(cr, marker_size, camera_matrix, camera_distortion)

            #-- Unpack the output, get only the first
            rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

            #-- Draw the detected marker and put a reference frame over it
            aruco.drawDetectedMarkers(frame, corners)
            aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)

            #-- Print the tag position in camera frame
            xa, ya, za = (tvec[0], -tvec[1], tvec[2])
            str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f"%(tvec[0], -tvec[1], tvec[2])
            cv2.putText(frame, str_position, (0, 100), font, 1, (255, 255, 255), 2, cv2.LINE_AA)

    #--- Display the frame
    cv2.imshow('frame', frame)
    #print(xa, ya, za)

    #--- use 'q' to quit
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        break
    a11, a22, a33, a44 = inverse_kinematics(xa - 8, ya + 35, 84 - za)
    # print(t_1, t_2, t_3)
    a11 = float("{0:.3f}".format(a11))
    a22 = float("{0:.3f}".format(a22))
    a33 = float("{0:.3f}".format(a33))
    a44 = float("{0:.3f}".format(a44))
    data = str(float(a11-90)) + ',' + str(float(a22)) + ',' + str(float(a33)) + ',' + str(float(a44-90)) + '\n'
    print(data)
    data = data.encode('utf-8')
    # print(data)
    try:
        arduino.write(data)
    except:
        print("No Connection")
    x_3, y_3, z_3, r_x_1, r_y_1, r_z_1, r_x_2, r_y_2, r_z_2, r_x_3, r_y_3, r_z_3 = forward_kinematics(a11, a22, a33,
                                                                                                      a44)

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
    plt.pause(0.000000000001)

    line1.remove()
    line2.remove()
    line3.remove()
    tgt.remove()


