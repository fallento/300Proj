# Servo control using gpiozero for GWS S03N STD on Raspberry Pi
# Continuously sweeps servo between 0° and 180° using GPIO 24
# Based on SCUTTLE motors.py style


import gpiozero                             # for PWM output
from gpiozero import PWMOutputDevice as pwm
import time                                 # for delays
import numpy as np                          # for angle conversions


# Constants
SERVO_PIN = 25                              # BCM numbering (Physical pin 18)
FREQ = 50                                   # 50 Hz for standard servo PWM
servo_pwm = pwm(SERVO_PIN, frequency=FREQ, initial_value=0)


def angle_to_duty(angle):
    """
    Converts an angle (0–180°) to a duty cycle (0 to 1) for 50 Hz PWM.
    1 ms = 5% duty → 0°, 2 ms = 10% duty → 180°
    """
    angle = max(0, min(180, angle))         # Clamp to valid range
    duty = 0.05 + (angle / 180.0) * 0.05     # Map to 0.05–0.10 (5%–10%)
    return round(duty, 3)


def set_servo_angle(angle):
    duty = angle_to_duty(angle)
    servo_pwm.value = duty
    time.sleep(0.5)
    servo_pwm.value = 0                     # Disable to reduce jitter


if __name__ == "__main__":
    while True:
        print("servo.py: setting to 0°")
        set_servo_angle(12)
        time.sleep(1)
        


        print("servo.py: setting to 180°")
        set_servo_angle(180)
        time.sleep(1)