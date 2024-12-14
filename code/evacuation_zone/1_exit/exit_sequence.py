from config import *
import RPi.GPIO as GPIO

def exit_sequence():
    print(f"Touches: {GPIO.input(front_touch_pins[0])} {GPIO.input(front_touch_pins[1])} {GPIO.input(back_touch_pins[0])} {GPIO.input(back_touch_pins[1])}", end="    |    ")