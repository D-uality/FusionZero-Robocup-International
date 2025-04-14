from config import *

import RPi.GPIO as GPIO

def initalise():
    for pin in touch_pins:
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def read(pins=touch_pins):
    values = []

    for pin in pins:
        values.append(GPIO.input(pin))

    print("touch:", values, end="    ")
    return values