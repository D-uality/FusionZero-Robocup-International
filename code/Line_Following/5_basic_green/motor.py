import time
from adafruit_servokit import ServoKit

pca = ServoKit(channels=16)

stop_speed = 95

def servo_write(left_speed, right_speed):
    left_speed = max(35, min(145, stop_speed + left_speed))
    right_speed = max(35, min(145, stop_speed - right_speed))

    pca.servo[14].angle = left_speed
    pca.servo[15].angle = left_speed
    pca.servo[12].angle = right_speed
    pca.servo[13].angle = right_speed