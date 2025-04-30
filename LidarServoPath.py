import L1_lidar as lidar
import L2_vector as vec
import L2_speed_control as sc
import L2_inverse_kinematics as ik
import numpy as np
import math
import time
import socket
import gpiozero                             # for PWM output
from gpiozero import PWMOutputDevice as pwm
import time                                 # for delays
import numpy as np                          # for angle conversions
import pigpio
import time
import logging

TARGET_HEADING_DEG = None
TARGET_DISTANCE_M = None
HAS_NEW_TARGET = False



# ---------------- Logging Setup ---------------- #
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S"
)

# ---------------- GPIO Pin Config ---------------- #
SERVOS = {
    "servo1": 4,    # GPIO4
    "servo2": 24,   # GPIO24 
    "servo3": 27,   # GPIO27
    "servo4": 25    # GPIO25
}

# CONSTANTS
FORWARD_SPEED = 0.4      # meters per second
STOP_DISTANCE_FRONT_M = 5    
FRONT_ANGLE_RANGE = 15   # degrees for "straight ahead"
SCAN_POINTS = 100        # number of lidar points
NODE_RED_IP = "127.0.0.1"
NODE_RED_PORT = 3555

# ---------------- Servo Constants (MG946R specific) ---------------- #
SERVO_FREQ = 50               # Servo frequency (50Hz standard)
SERVO_MIN_US = 1000            # Minimum pulse width (µs) ~ 0°
SERVO_MAX_US = 2200            # Maximum pulse width (µs) ~ 180°

# Initialize UDP socket for Node-RED dashboard
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
SERVO_CONTROL_PORT = 3556  # different from LIDAR send port
recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
recv_sock.bind(("0.0.0.0", SERVO_CONTROL_PORT))
recv_sock.setblocking(False)  # non-blocking receive

# Helper function to format lidar points for Node-RED
def format_lidar_points(scan):
    rows = ''
    for d, t in scan:
        x, y = vec.polar2cart(d, t)
        rows += '{x: ' + str(x) + ', y: ' + str(y) + ', r:3},'
    return rows[:-1]


#new Servo stuff
pi = pigpio.pi()
logging.info("Motors initialized with hardware PWM at 50% speed.")

# Initialize servos
for name, gpio in SERVOS.items():
    pi.set_mode(gpio, pigpio.OUTPUT)
    if name == "servo1" or name == "servo2":
        pi.set_servo_pulsewidth(gpio, 1000)
    else:
        pi.set_servo_pulsewidth(gpio, 1050)  # Center position (~10°)
    logging.info(f"{name} initialized at 10° (1111µs).")

# ---------------- Helper Functions ---------------- #
def angle_to_pulsewidth(angle):
    """
    Converts an angle (0-180 degrees) to a microsecond pulse width
    for MG946R servos.
    """
    angle = max(0, min(180, angle))  # Clamp to valid range
    
    return int(SERVO_MIN_US + (angle / 180.0) * (SERVO_MAX_US - SERVO_MIN_US))

def set_servo_angle(servo_name, angle):
    """
    Set a specific servo to an angle between 0-180°.
    """
    if servo_name not in SERVOS:
        logging.warning(f"Servo {servo_name} not recognized.")
        return

    pw = angle_to_pulsewidth(angle)
    
    pi.set_servo_pulsewidth(SERVOS[servo_name], pw)
    logging.info(f"Set {servo_name} to {angle}° → {pw}µs pulse width.")

#open Servo
def ServoOpen(command):
    name = (command[4:])
    if name not in SERVOS:
        print("Invalid servo name.")
    angle = 180
    set_servo_angle(name, angle)
    time.sleep(1)  # 1-second delay between moves

#close Servo
def ServoClose(command):
    name = (command[5:])
    if name not in SERVOS:
        print("Invalid servo name.")
    if name == "servo1" or name == "servo2":
        angle = 0
    else:
        angle = 10
    set_servo_angle(name, angle)
    time.sleep(1)  # 1-second delay between moves
#close all
def allClose():
    for angle in [10]:
                for name in SERVOS:
                    if name == "servo1" or name == "servo2":
                        angle = 0
                    set_servo_angle(name, angle)
                time.sleep(1)  # 1-second delay between moves

#open all
def allOpen():
    for angle in [180]:
                for name in SERVOS:
                    set_servo_angle(name, angle)
                time.sleep(1)  # 1-second delay between moves

#END OF SERVO SHI
degree = -1
distance = 0
# Main Loop
import select
ready_to_move = False



while degree == -1 or distance == 0:
    timeout_seconds=0.1
    ready = select.select([recv_sock], [], [], timeout_seconds)
    if ready[0]:
        data, addr = recv_sock.recvfrom(1024)
        command = data.decode("utf-8").strip().lower()
        print(f"Received command: {command}")

        if command[:6] == "degree":
            degree = int(command[7:])
        elif command[:8] == "distance":
            distance = int(command[9:])
        elif command.startswith("open"):
            ServoOpen(command)
        elif command.startswith("close"):
            ServoClose(command)
        elif command == "allopen":
            allOpen()
        elif command == "allclose":
            allClose()
    if degree != -1 and distance != 0:
        ready_to_move = True

while degree > -1 and distance > 0:
    scan = lidar.polarScan(SCAN_POINTS)

    # Filter valid points
    valids = vec.getValid(scan)

    # Print distances and angles detected by LIDAR
    print("LIDAR Scan Data:")
    #for distance_m, angle_deg in valids:
    #    print(f"Distance: {distance_m:.3f} m, Angle: {angle_deg:.2f} degrees")

    # Send formatted lidar data to Node-RED
    message = format_lidar_points(valids).encode('utf-8')
    sock.sendto(message, (NODE_RED_IP, NODE_RED_PORT))

    # Find nearest obstacle
    nearest_point = vec.nearest(valids)
    distance_m = nearest_point[0]
    angle_deg = nearest_point[1]

    # Print nearest point
    print(f"Nearest obstacle: {distance_m:.3f} m at {angle_deg:.2f} degrees")

    obstacle_action = None
    
    if distance_m <= STOP_DISTANCE_FRONT_M:
        if -FRONT_ANGLE_RANGE <= angle_deg <= FRONT_ANGLE_RANGE:
            obstacle_action = "reverse_turn"

    if obstacle_action:
        print("Obstacle detected. Stopping.")
        sc.driveOpenLoop([0, 0])
        time.sleep(0.5)

        if obstacle_action == "reverse_turn":
            print("Obstacle straight ahead. Reversing and turning.")
            sc.driveOpenLoop(ik.getPdTargets([-FORWARD_SPEED, -FORWARD_SPEED])) # ASSUMING: reverses
            time.sleep(1)
            sc.driveOpenLoop(ik.getPdTargets([0, 20])) # ASUSMING: turns
            time.sleep(0.5)

    else:
        # No obstacle detected, drive forward
        print("No obstacle detected.")
        wheel_speeds = ik.getPdTargets([FORWARD_SPEED, 0])
        """
        Move the robot in a given direction (degrees) for a given distance (meters).
        """
        radians = math.radians(degree)
        # Calculate X and Y motion components
        x = math.cos(radians) * FORWARD_SPEED
        y = math.sin(radians) * FORWARD_SPEED

        # Compute wheel speeds
        wheel_speeds = ik.getPdTargets([x, y])
        duration = distance / FORWARD_SPEED  # time = distance / speed

        logging.info(f"Moving to {distance}m at {degree}° (for {duration:.2f}s)")
        sc.driveOpenLoop(wheel_speeds)
        time.sleep(duration)
        sc.driveOpenLoop([0, 0])
        logging.info("Movement complete.")

    try:
        data, addr = recv_sock.recvfrom(1024)
        command = data.decode("utf-8").strip().lower()
        print(f"Received command: {command}")

        if command[:4] == "open":
            ServoOpen(command)
        elif command[:5] == "close":
            ServoClose(command)
        elif command == "allopen":
            allOpen()
        elif command == "allclose":
            allClose()
        
    except BlockingIOError:
        pass  # No message received, continue loop

    time.sleep(0.1)  # Small delay to prevent overload




