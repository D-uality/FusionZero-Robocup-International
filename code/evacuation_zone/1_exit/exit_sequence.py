import RPi.GPIO as GPIO

from config import *
import motors
import touch_sensors
import laser_sensors

def exit_sequence():
    # touch_sensors.read()
    # motors.run(30, 30)

    motors.run_until(20, 20, touch_sensors.read, 0, "==", 0)

    motors.run(0, 0, 1)