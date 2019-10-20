import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import serial
import cv2
import cv2.aruco as aruco
import time, math

cap = cv2.VideoCapture(1)
marker_size  = 5.3#- [cm]
xa, ya, za = 40, 40, 40
follow = False
magnet = 1
#######################################################################################################################
try:
    port = 'COM8'  # "/dev/tty.HC-06-DevB"
    print("Connecting to device connected to " + port)
    arduino = serial.Serial(port, 9600, timeout=.1)
    print("Connected")
except:
    print("Not connected")

time.sleep(5)
temp = 0
last = 0

fig = plt.figure(1)
ar = fig.gca(projection='3d')


ar.set_xlim(-150, 150)
ar.set_ylim(-150, 150)
ar.set_zlim(-150, 150)


r1 = 22.5
r2 = 17
r3 = 5
R = r1 + r2 + r3


def constrain(amt, low, high):
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


calib_path = ""
camera_matrix   = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')
R_flip = np.zeros((3, 3), dtype=np.float32)
R_flip[0, 0] = 1.0
R_flip[1, 1] = -1.0
R_flip[2, 2] = -1.0

aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters  = aruco.DetectorParameters_create()
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
font = cv2.FONT_HERSHEY_PLAIN

while True:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters,cameraMatrix=camera_matrix, distCoeff=camera_distortion)
    if ids is not None:
        print('ID:' , ids)
        for cr in corners:
            ret = aruco.estimatePoseSingleMarkers(cr, marker_size, camera_matrix, camera_distortion)
            rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]
            aruco.drawDetectedMarkers(frame, corners)
            aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)
            xa, ya, za = (tvec[0], -tvec[1], tvec[2])
            str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f"%(tvec[0], -tvec[1], tvec[2])
            cv2.putText(frame, str_position, (0, 100), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.imshow('frame', frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        break
    if key == ord('f'):
        if follow:
            print("Follow_False")
            follow = False
        else:
            follow = True
            print("Follow_True")
    if key == ord('m'):
        if magnet == 1:
            print("Magnet_deactive")
            magnet = 0
        else:
            magnet = 1
            print("Magnet_active")
    if key == ord('h'):
        print("Drop_position")

        a11, a22, a33, a44 = inverse_kinematics(-20 - 9, -15 + 35, 87 - 90)

        a11 = a11 - 90
        a11 = float("{0:.3f}".format(a11))
        a22 = float("{0:.3f}".format(a22))
        a33 = float("{0:.3f}".format(a33))
        a44 = float("{0:.3f}".format(a44))
        data = str(float(a11)) + ',' + str(float(a22)) + ',' + str(float(a33)) + ',' + str(float(a44 - 90)) + ',' + str(
            float(1)) + '\n'
        data = data.encode('utf-8')
        try:
            print(data)
            arduino.write(data)
        except:
            print("No Connection")
        time.sleep(1)
        data = str(float(a11)) + ',' + str(float(a22)) + ',' + str(float(a33)) + ',' + str(float(a44 - 90)) + ',' + str(
            float(0)) + '\n'
        data = data.encode('utf-8')
        try:
            print(data)
            arduino.write(data)
        except:
            print("No Connection")
        follow = False
    if key == ord('j'):
        print("Home_position_2")
        a11, a22, a33, a44 = inverse_kinematics(36- 9,  -10+ 35, 0)

        a11 = a11 - 90
        a11 = float("{0:.3f}".format(a11))
        a22 = float("{0:.3f}".format(a22))
        a33 = float("{0:.3f}".format(a33))
        a44 = float("{0:.3f}".format(a44))
        data = str(float(a11)) + ',' + str(float(a22)) + ',' + str(float(a33)) + ',' + str(float(a44 - 90)) + ',' + str(
            float(1)) + '\n'
        data = data.encode('utf-8')
        try:
            print(data)
            arduino.write(data)
        except:
            print("No Connection")
        time.sleep(1)
        data = str(float(a11)) + ',' + str(float(a22)) + ',' + str(float(a33)) + ',' + str(float(a44 - 90)) + ',' + str(
            float(0)) + '\n'
        data = data.encode('utf-8')
        try:
            print(data)
            arduino.write(data)
        except:
            print("No Connection")

    a11, a22, a33, a44 = inverse_kinematics(xa - 2, ya + 35, 86- za)
    a11 = a11-90
    a11 = float("{0:.3f}".format(a11))
    a22 = float("{0:.3f}".format(a22))
    a33 = float("{0:.3f}".format(a33))
    a44 = float("{0:.3f}".format(a44))
    data = str(float(a11)) + ',' + str(float(a22)) + ',' + str(float(a33)) + ',' + str(float(a44-90)) + ',' + str(float(magnet)) + '\n'
    data = data.encode('utf-8')
    #print(data)
    if key == ord('s') or follow:
        try:
            print(data)
            arduino.write(data)
        except:
            print("No Connection")
    x_3, y_3, z_3, r_x_1, r_y_1, r_z_1, r_x_2, r_y_2, r_z_2, r_x_3, r_y_3, r_z_3 = forward_kinematics(a11, a22, a33, a44)
    line1, = ar.plot(r_x_1, r_y_1, r_z_1, 'r-*')
    line2, = ar.plot(r_x_2, r_y_2, r_z_2, 'g-*')
    line3, = ar.plot(r_x_3, r_y_3, r_z_3, 'b-*')
    tgt = ar.scatter(x_3, y_3, z_3, color='black')
    ar.set_xlabel('X - axis')
    ar.set_ylabel('Y - axis')
    ar.set_zlabel('Z - axis')
    plt.pause(0.000000000001)
    line1.remove()
    line2.remove()
    line3.remove()
    tgt.remove()



