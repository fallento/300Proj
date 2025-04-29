import L1_lidar as lidar
import L2_vector as vec
import L2_speed_control as sc
import L2_inverse_kinematics as ik
import numpy as np
import math
import time
import socket

# CONSTANTS
FORWARD_SPEED = 0.4      # meters per second
STOP_DISTANCE_FRONT_M = 0.58    # 5 inches in meters
FRONT_ANGLE_RANGE = 15   # degrees for "straight ahead"
SCAN_POINTS = 100        # number of lidar points
NODE_RED_IP = "127.0.0.1"
NODE_RED_PORT = 3555

# Initialize UDP socket for Node-RED dashboard
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Helper function to format lidar points for Node-RED
def format_lidar_points(scan):
    rows = ''
    for d, t in scan:
        x, y = vec.polar2cart(d, t)
        rows += '{x: ' + str(x) + ', y: ' + str(y) + ', r:3},'
    return rows[:-1]

# Main Loop
while True:
    scan = lidar.polarScan(SCAN_POINTS)

    # Filter valid points
    valids = vec.getValid(scan)

    # Print distances and angles detected by LIDAR
    print("LIDAR Scan Data:")
    for distance_m, angle_deg in valids:
        print(f"Distance: {distance_m:.3f} m, Angle: {angle_deg:.2f} degrees")

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
        #elif -75 <= angle_deg <= -FRONT_ANGLE_RANGE:
         #   obstacle_action = "turn_left"
        #elif FRONT_ANGLE_RANGE <= angle_deg <= 75:
         #   obstacle_action = "turn_right"

    if obstacle_action:
        print("Obstacle detected. Stopping.")
        sc.driveOpenLoop([0, 0])
        time.sleep(0.5)

        if obstacle_action == "reverse_turn":
            print("Obstacle straight ahead. Reversing and turning.")
            sc.driveOpenLoop(ik.getPdTargets([-FORWARD_SPEED, -FORWARD_SPEED]))
            time.sleep(1)
            sc.driveOpenLoop(ik.getPdTargets([0, 5]))
            time.sleep(0.5)

        #elif obstacle_action == "turn_left":
         #   print("Obstacle on right. Turning left.")
          #  sc.driveOpenLoop(ik.getPdTargets([0, 3.5]))
           # time.sleep(0.5)

        #elif obstacle_action == "turn_right":
         #   print("Obstacle on left. Turning right.")
          #  sc.driveOpenLoop(ik.getPdTargets([0, -3.5]))
           # time.sleep(0.5)

    else:
        # No obstacle detected, drive forward
        print("No obstacle detected.")
        wheel_speeds = ik.getPdTargets([FORWARD_SPEED, 0])
        sc.driveOpenLoop(wheel_speeds)

    time.sleep(0.1)  # Small delay to prevent overload
