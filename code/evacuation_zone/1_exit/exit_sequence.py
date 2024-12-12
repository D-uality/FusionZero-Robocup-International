import RPi.GPIO as GPIO

from config import *
from motor import *
from touch_sensors import *

def exit_sequence():

    read_touch()
    run_motors(50, 50)