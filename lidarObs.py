import L1_lidar as lidar
import L2_vector as vec
import L2_speed_control as sc
import L2_inverse_kinematics as ik
import numpy as np
import socket
import pigpio
from time import sleep

# Constants
STOP_DISTANCE_FRONT_M = 1.5
FRONT_ANGLE_RANGE = 15
FORWARD_SPEED = 0.3
TURN_SPEED = 0.5
SCAN_POINTS = 100

# Servo setup
SERVOS = {
    "servo1": 4,
    "servo2": 24,
    "servo3": 27,
    "servo4": 25
}
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

# UDP for commands
SERVO_CONTROL_PORT = 3556
recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
recv_sock.bind(("0.0.0.0", SERVO_CONTROL_PORT))
recv_sock.setblocking(False)

def check_for_obstacle():
    scan = lidar.polarScan(SCAN_POINTS)
    valids = vec.getValid(scan)
    nearest_point = vec.nearest(valids)
    distance_m, angle_deg = nearest_point
    return distance_m, angle_deg

print("SCUTTLE running: obstacle avoidance mode active.")

# --- MOVEMENT LOOP ---
user_controlled = False
while not user_controlled:
    try:
        data, _ = recv_sock.recvfrom(1024)
        command = data.decode("utf-8").strip().lower()
        print(f"Received user command: {command}")
        sc.driveOpenLoop([0, 0])  # Stop movement
        user_controlled = True    # Switch to manual mode

        # Handle command before breaking
        if command.startswith("open"):
            ServoOpen(command)
        elif command.startswith("close"):
            ServoClose(command)
        elif command == "allopen":
            allOpen()
        elif command == "allclose":
            allClose()
        break  # Exit movement loop
    except BlockingIOError:
        pass

    # Obstacle avoidance
    distance_m, angle_deg = check_for_obstacle()
    print(f"Nearest obstacle: {distance_m:.2f} m at {angle_deg:.1f}°")

    if distance_m <= STOP_DISTANCE_FRONT_M and -FRONT_ANGLE_RANGE <= angle_deg <= FRONT_ANGLE_RANGE:
        print("Obstacle detected! Stopping.")
        sc.driveOpenLoop([0, 0])
        sleep(0.3)

        print("Reversing...")
        sc.driveOpenLoop(ik.getPdTargets([-FORWARD_SPEED, 0]))
        sleep(1.0)

        turn_direction = TURN_SPEED + 20 if angle_deg < 0 else -TURN_SPEED + 20
        print(f"Turning {'left' if turn_direction > 0 else 'right'}...")
        sc.driveOpenLoop(ik.getPdTargets([0, turn_direction]))
        sleep(1.0)

        print("Resuming forward...")
        sc.driveOpenLoop(ik.getPdTargets([0.95, 0]))
        sleep(1.0)

        print(f"Turning {'left' if turn_direction > 0 else 'right'} again...")
        sc.driveOpenLoop(ik.getPdTargets([0, -turn_direction]))
        sleep(1.0)

        print("Resuming forward...")
        sc.driveOpenLoop(ik.getPdTargets([FORWARD_SPEED, 0]))
        sleep(0.2)
    else:
        sc.driveOpenLoop(ik.getPdTargets([FORWARD_SPEED, 0]))

    sleep(0.1)

# --- MANUAL COMMAND LOOP ---
print("Manual control mode. Waiting for more Node-RED commands...")
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
