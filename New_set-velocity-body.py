####Dependencies###################
import math
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import socket
import exceptions
import argparse
import cv2
import cv2.aruco as aruco


###Function definitions for mission####

##Function to connect script to drone
def connectMyCopter():
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()

    connection_string = "127.0.0.1:5760"

    vehicle = connect(connection_string, wait_ready=True)

    return vehicle

def arm_and_takeoff_nogps(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude without GPS data.
    """

    ##### CONSTANTS #####
    DEFAULT_TAKEOFF_THRUST = 0.7
    SMOOTH_TAKEOFF_THRUST = 0.6

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    # If you need to disable the arming check,
    # just comment it with your own responsibility.
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)


    print("Arming motors")
    # Copter should arm in GUIDED_NOGPS mode
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        vehicle.armed = True
        time.sleep(1)

        print("Taking off!")
    thrust = DEFAULT_TAKEOFF_THRUST
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        if current_altitude >= aTargetAltitude*0.95: # Trigger just below target alt.
            set_attitude_target(thrust = 0.5)
            break
        elif current_altitude >= aTargetAltitude*0.6:
            thrust = SMOOTH_TAKEOFF_THRUST
        set_attitude(thrust = thrust)
        time.sleep(0.2)
                                        


def camera_to_uav(x_cam, y_cam):
    x_uav = -y_cam
    y_uav = x_cam
    return (x_uav, y_uav)


def uav_to_ne(x_uav, y_uav, yaw_rad):
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)

    north = x_uav * c - y_uav * s
    east = x_uav * s + y_uav * c
    return (north, east)

def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]


#### Moving mechanics
def send_attitude_target(roll_angle = 0.0, pitch_angle = 0.0,yaw_angle = None,
                         yaw_rate = 0.0, use_yaw_rate = False,thrust = 0.5):
    """
    use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
                  When one is used, the other is ignored by Ardupilot.
    thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
            Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
            the code for maintaining current altitude.
    """
    if yaw_angle is None:
        # this value may be unused by the vehicle, depending on use_yaw_rate
        yaw_angle = vehicle.attitude.yaw
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_target_encode(
                                                    0, # time_boot_ms
                                                    1, # Target system
                                                    1, # Target component
                                                    0b00000000 if use_yaw_rate else 0b00000100,
                                                    to_quaternion(roll_angle, pitch_angle, yaw_angle), # Quaternion
                                                    0, # Body roll rate in radian
                                                    0, # Body pitch rate in radian
                                                    math.radians(yaw_rate), # Body yaw rate in radian/second
                                                    thrust  # Thrust
                                                )
    vehicle.send_mavlink(msg)

def track():
    marker_found = False

    ret,frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, rejected = aruco.detectMarker(img = gray, dictionary = aruco_dict, parameters = parameters, cameraMatrix = camera_matrix, distCoeff=camera_distortion)
    if ids is not None and ids[0] == id_to_find:
        ret = aruco.estimatePoseSingleMakers(corners, marker_size, camera_matri, camera_distortion)

        rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

        R_ct = np.matrix(cv2.Rodrigue(rvec)[0])
        R_tc = R_ct.T

        roll_maker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_ct)
        roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip*R_tc)

        x = tvec[0]
        y = tvec[1]
        z = tvec[2]
        yaw = math.degrees(yaw_camera)

        return marker_found, x, y, z, yaw
        
    
    

#### Mission################################

vehicle=connectMyCopter()
print("About to takeoff..")



#--------------------------------------------------
#-------------- PARAMETERS
#--------------------------------------------------
rad_2_deg   = 180.0/math.pi
deg_2_rad   = 1.0/rad_2_deg

#--------------------------------------------------
#-------------- LANDING MARKER
#--------------------------------------------------
#--- Define Tag
id_to_find      = 72
marker_size     = 10 #- [cm]

aruco_dict = aruco.gerPredefinedDictionary(aruco.DICT_ARUCO_ORIGIN)
parameter = aruco.DetectorParameters_create()

calib_path          = cwd+"/../opencv/"
camera_matrix       = np.loadtxt(calib_path+'cameraMatrix_raspi.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_raspi.txt', delimiter=',')
aruco_tracker       = ArucoSingleTracker(id_to_find=id_to_find, marker_size=marker_size, show_video=False,
                camera_matrix=camera_matrix, camera_distortion=camera_distortion)

cap = cv2.VideoCapture(0)
count = 0
yaw_is_correct = False
pos_is_correct = False
last_valo = 0
last_time = 0
last_distance = 0
k_distance = 10/75 ## max angle 10 deg is devided by max distance the camera should recognize marker at 2 m high 75 cm
k_velocity = 1
k_acceleration = 1
k_yaw = 10/90 ## max 10 deg/sec is devided by max 90(-90) deg 
start_yaw_time = time.time()
yaw_time = 0

while True:
    
    ### preventing drone from fly away
    if count == 3:
        send_attitude_target(0,0,0,0, False,0.5)
        count = 0
    ### general code
    marker_found, x_cm, y_cm, __, yaw = track()
    if marker_found:
        x_cm, y_cm = camera_to_uav(x_cm, y_cm)
        distance = math.sqrt((x_cm**2)+(y_cm**2))
        count = 0
             
    else:
        count += 1
        continue

    # change to yew_rate = f(yaw);linear or 2nd order
    yaw = math.acos(x)
    
    if not yaw_is_correct or math.abs(yaw) >= 1:
        yaw_is_correct = False
        yaw_rate = k_yaw * yaw
        
        if yaw_rate * yaw_time >= yaw:
            yaw_rate = math.ceil(yaw_rate * 0.5)
        
        send_attitude_target(0,0,0,yaw_rate,True,0.5)
        yaw_time = time.time() - start_yaw_time
        start_yaw_time = time.time()
        
    else:
        yaw_is_correct = True

    if (not pos_is_correct or distance >= 10) and yaw_is_correct:
        pos_is_correct = False
        velocity = (distance - last_distance)/pitch_time
        acceleration = (velocity - last_velocity)/pitch_time
        angle = k_distance * distance - k_velocity * velocity - k_acceleration * acceleration
        last_distance = distance
        last_velocirt = velocity
        
    elif yaw_is_correct:
        pos_is_correct = True
        continue
    
    else:continue


    
    start_t = time.time()
    while time.time()-start_t <= 0.1:
        send_attitude_target(0,angle,0,0,False,0.5)
    send_attitude_target(0,0,0,0,False,0.5)
    pitch_time = time.time() - start_t
