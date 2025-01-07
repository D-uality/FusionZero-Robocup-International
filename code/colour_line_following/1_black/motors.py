import time
from adafruit_servokit import ServoKit

pca = ServoKit(channels=16)

stop_speed = 95
max_speed = 70

def run(left_speed, right_speed, delay = 0):
    """
    Drives 4 motors on car. Positive values move forwards, negatives move backwards, 0 is stationary.

    :param left_speed: Takes values 0 to max_speed and drives left motors.
    :param right_speed: Takes values 0 to max_speed and drives right motors.
    :param delay: Time delay (secounds) after writing motor speed. Automatically set to 0 if no input.
    """
    left_speed = max(90-max_speed, min(90+max_speed, stop_speed + left_speed))
    right_speed = max(90-max_speed, min(90+max_speed, stop_speed - right_speed))

    pca.servo[14].angle = left_speed
    pca.servo[15].angle = left_speed
    pca.servo[12].angle = right_speed
    pca.servo[13].angle = right_speed

    time.sleep(delay)