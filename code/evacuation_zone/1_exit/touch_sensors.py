from config import *

import RPi.GPIO as GPIO

def initalise_touch():
    for pin in touch_pins:
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def read_touch(pins=touch_pins):
    values = []

    for pin in pins:
        values.append(GPIO.input(pin))

    print(values, end="    ")