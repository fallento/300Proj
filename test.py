# Multi-servo (MG946R) and motor control using pigpio on Raspberry Pi 4
# Supports 4 MG946R servos + 2 motors (via hardware PWM)

import pigpio
import time
import logging

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



# ---------------- Servo Constants (MG946R specific) ---------------- #
SERVO_FREQ = 50               # Servo frequency (50Hz standard)
SERVO_MIN_US = 1000            # Minimum pulse width (µs) ~ 0°
SERVO_MAX_US = 2200            # Maximum pulse width (µs) ~ 180°

# ---------------- Motor Constants ---------------- #
MOTOR_FREQ = 10000             # Motor PWM frequency: 10kHz
DEFAULT_MOTOR_DUTY = 500000    # 50% duty cycle (0–1,000,000 scale)

# ---------------- Pi Initialization ---------------- #
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("Unable to connect to pigpio daemon")

# Initialize motors
#pi.set_mode(MOTOR_PWM_A, pigpio.OUTPUT)
#pi.set_mode(MOTOR_PWM_B, pigpio.OUTPUT)

#pi.hardware_PWM(MOTOR_PWM_A, MOTOR_FREQ, DEFAULT_MOTOR_DUTY)
#pi.hardware_PWM(MOTOR_PWM_B, MOTOR_FREQ, DEFAULT_MOTOR_DUTY)
logging.info("Motors initialized with hardware PWM at 50% speed.")

# Initialize servos
for name, gpio in SERVOS.items():
    pi.set_mode(gpio, pigpio.OUTPUT)
    pi.set_servo_pulsewidth(gpio, 1500)  # Center position (~90°)
    logging.info(f"{name} initialized at 90° (1500µs).")

# ---------------- Helper Functions ---------------- #
def angle_to_pulsewidth(angle):
    """
    Converts an angle (0–180 degrees) to a microsecond pulse width
    for MG946R servos.
    """
    angle = max(0, min(180, angle))  # Clamp to valid range
    return int(SERVO_MIN_US + (angle / 180.0) * (SERVO_MAX_US - SERVO_MIN_US))

def set_servo_angle(servo_name, angle):
    """
    Set a specific servo to an angle between 0–180°.
    """
    if servo_name not in SERVOS:
        logging.warning(f"Servo {servo_name} not recognized.")
        return

    pw = angle_to_pulsewidth(angle)
    pi.set_servo_pulsewidth(SERVOS[servo_name], pw)
    logging.info(f"Set {servo_name} to {angle}° → {pw}µs pulse width.")

# ---------------- Main Execution ---------------- #
if __name__ == "__main__":
    try:
        while True:
            # Sweep all servos 0° → 90° → 180° → 90° continuously
            for angle in [12, 180]:
                for name in SERVOS:
                    set_servo_angle(name, angle)
                time.sleep(1)  # 1-second delay between moves

    except KeyboardInterrupt:
        logging.info("Shutting down gracefully...")

    finally:
        # Stop servos
        for gpio in SERVOS.values():
            pi.set_servo_pulsewidth(gpio, 0)

        # Stop motors
        #pi.hardware_PWM(MOTOR_PWM_A, 0, 0)
        #pi.hardware_PWM(MOTOR_PWM_B, 0, 0)

        # Disconnect pigpio
        pi.stop()
        logging.info("GPIO cleanup complete. Program exited.")
