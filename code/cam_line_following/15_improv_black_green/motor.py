import time
from adafruit_servokit import ServoKit

pca = ServoKit(channels=16)

stop_speed = 95
max_speed = 70

def servo_write(left_speed, right_speed):
    left_speed = max(90-max_speed, min(90+max_speed, stop_speed + left_speed))
    right_speed = max(90-max_speed, min(90+max_speed, stop_speed - right_speed))

    pca.servo[14].angle = left_speed
    pca.servo[15].angle = left_speed
    pca.servo[12].angle = right_speed
    pca.servo[13].angle = right_speed