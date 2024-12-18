from config import *

import RPi.GPIO as GPIO
import oled_display

def initalise():
    try:
        for touch_pin in touch_pins:
            GPIO.setup(touch_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        print("Touch Initalised!")
        oled_display.text("Touch: ✓", 0, 30)
        
    except Exception as e:
        print(f"Error initalisng touch: {e}")
        oled_display.text("Touch: x", 0, 30)

def read(pins=touch_pins):
    values = []

    for pin in touch_pins:
        values.append(GPIO.input(pin))

    print(f"Touch: {values}", end="    ")
    return values