import RPi.GPIO as GPIO

from config import *
import motors
import touch_sensors
import laser_sensors

def exit_sequence():

    touch_sensors.read()
    laser_sensors.read()
    motors.run(30, 30)