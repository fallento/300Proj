import L1_lidar as lidar
import L2_vector as vec
import L2_speed_control as sc
import L2_inverse_kinematics as ik
import numpy as np
import socket
import pigpio
import logging
import math
from time import sleep
hold_deg = True
hold_dist = True
# Constants
STOP_DISTANCE_FRONT_M = 0.75     # ⬅ updated threshold
FRONT_ANGLE_RANGE = 15           # degrees
FORWARD_SPEED = 0.3              # path speed
SCAN_POINTS = 100

# Servo setup
SERVOS = {
    "servo1": 4,
    "servo2": 24,
    "servo3": 27,
    "servo4": 25
}
SERVO_FREQ = 50
SERVO_MIN_US = 1000
SERVO_MAX_US = 2200

pi = pigpio.pi()
for name, gpio in SERVOS.items():
    pi.set_mode(gpio, pigpio.OUTPUT)
    if name in ["servo1", "servo2"]:
        pi.set_servo_pulsewidth(gpio, 1000)
    else:
        pi.set_servo_pulsewidth(gpio, 1050)

def angle_to_pulsewidth(angle):
    angle = max(0, min(180, angle))
    return int(SERVO_MIN_US + (angle / 180.0) * (SERVO_MAX_US - SERVO_MIN_US))

def set_servo_angle(servo_name, angle):
    if servo_name in SERVOS:
        pw = angle_to_pulsewidth(angle)
        pi.set_servo_pulsewidth(SERVOS[servo_name], pw)

def ServoOpen(command):
    name = command[4:]
    if name in SERVOS:
        set_servo_angle(name, 180)
        sleep(1)

def ServoClose(command):
    name = command[5:]
    if name in SERVOS:
        angle = 0 if name in ["servo1", "servo2"] else 10
        set_servo_angle(name, angle)
        sleep(1)

def allOpen():
    for name in SERVOS:
        set_servo_angle(name, 180)
    sleep(1)

def allClose():
    for name in SERVOS:
        angle = 0 if name in ["servo1", "servo2"] else 10
        set_servo_angle(name, angle)
    sleep(1)

# Setup UDP for command reception
SERVO_CONTROL_PORT = 3556
recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
recv_sock.bind(("0.0.0.0", SERVO_CONTROL_PORT))
recv_sock.setblocking(False)
degree = 0
distance = 0
# Path motions
import select
motions=[]
def set_polar_cord(degree,distance):
    global motions
    angle_rad = math.radians(degree)
    turn_speed = 0.5  # rad/s (adjust based on your robot's ability)
    turn_time = round(abs(angle_rad) / turn_speed, 2)
    turn_direction = math.copysign(turn_speed, angle_rad)

    forward_speed = round(distance / 4, 2)  # keep this

    motions = [
        [0.0, turn_direction, turn_time],  # angular motion
        [forward_speed, 0.0, 4]            # forward motion
    ]
def check():
    global hold_deg, hold_dist, degree, distance
    while hold_deg == True or hold_dist == True:
        timeout_seconds=0.1
        ready = select.select([recv_sock], [], [], timeout_seconds)
        if ready[0]:
            data, _ = recv_sock.recvfrom(1024)
            command = data.decode("utf-8").strip().lower()
            print(f"Received command: {command}")
            if command.startswith("degree:"):
                degree = int(command[7:])
                print(f"Received command: {degree}")
                hold_deg = False
            elif command.startswith("distance:"):
                distance = int(command[9:])
                print(f"Received command: {distance}")
                hold_dist = False
            elif command.startswith("open"):
                ServoOpen(command)
            elif command.startswith("close"):
                ServoClose(command)
            elif command == "allopen":
                allOpen()
            elif command == "allclose":
                allClose()
        if hold_deg == False and hold_dist == False:
            set_polar_cord(degree,distance)
            execute(motions)
            hold_deg = True
            hold_dist = True

            


    
'''motions = [
    [0.3, 0.0, 2],
    [0.0, 0.785, 2],
    [0.3, 0.0, 2],
    [0.0, 0.785, 2],
    [0.3, 0.0, 2],
    [0.0, -0.788, 2],
    [0.3, 0.0, 2],
    [0.0, -0.788, 2],
    [0.3, 0.0, 2],
]'''

def check_for_obstacle():
    scan = lidar.polarScan(SCAN_POINTS)
    valids = vec.getValid(scan)
    nearest_point = vec.nearest(valids)
    distance_m, angle_deg = nearest_point

    if distance_m <= STOP_DISTANCE_FRONT_M and -FRONT_ANGLE_RANGE <= angle_deg <= FRONT_ANGLE_RANGE:
        return True
    return False

# Execute motions
def execute(motions):
    for count, motion in enumerate(motions):
        print(f"Starting Motion {count+1}")
        remaining_time = motion[2]
        wheel_speeds = ik.getPdTargets(motion[:2])
        sc.driveOpenLoop(wheel_speeds)

        start = sleep_time = 0.0
        step = 0.1

        while sleep_time < remaining_time:
            if check_for_obstacle():
                print("Obstacle detected! Stopping.")
                sc.driveOpenLoop([0, 0])
                sleep(0.5)

                # Reverse and turn
                sc.driveOpenLoop(ik.getPdTargets([-FORWARD_SPEED, 0]))
                sleep(1)
                #sc.driveOpenLoop(ik.getPdTargets([0, 0.5]))
                #sleep(0.5)

                # Wait until clear
                print("Waiting for obstacle to clear...")
                while check_for_obstacle():
                    sleep(0.1)

                print("Path clear. Resuming motion.")
                sc.driveOpenLoop(wheel_speeds)
            else:
                sleep(step)
                sleep_time += step
    # Stop after motions
    sc.driveOpenLoop([0, 0])
    print("All motions complete. Waiting for servo commands...")

check()

# Wait for UDP servo commands
while True:
    try:
        data, _ = recv_sock.recvfrom(1024)
        command = data.decode("utf-8").strip().lower()
        print(f"Received command: {command}")
        if command.startswith("open"):
            ServoOpen(command)
        elif command.startswith("close"):
            ServoClose(command)
        elif command == "allopen":
            allOpen()
        elif command == "allclose":
            allClose()
        
    except BlockingIOError:
        pass
    sleep(0.1)
