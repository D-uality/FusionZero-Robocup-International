import RPi.GPIO as GPIO
import time
import subprocess
import os
import sys

import cv2
import numpy as np
from random import randint
from picamera2 import Picamera2
from libcamera import Transform

current_dir = os.path.dirname(os.path.abspath(__file__))
modules_dir = os.path.abspath(os.path.join(current_dir, 'modules'))
if modules_dir not in sys.path: sys.path.insert(0, modules_dir)

import config
import camera
import motors
import touch_sensors
import laser_sensors
import led
import colour
import oled_display
import gyroscope

switch_pin = 22

GPIO.setmode(GPIO.BCM)
GPIO.setup(switch_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

running = False
main_process = None

try:
    oled_display.initialise()
    
    oled_display.reset()
    oled_display.text(f"READY TO BEGIN", 0, 0, size=10)
    
    print("Ready to begin!\n\n")
    while True:
        running = True if GPIO.input(switch_pin) == GPIO.LOW else False

        # If running is True and the competition script is not running, start it.
        if running and main_process is None:
            oled_display.reset()
            oled_display.text("STARTING", 0, 0, size=10)
            
            print("Starting competition_main.py...")
            # Launch the competition_main.py script
            main_process = subprocess.Popen(["python3", "competition_main.py"])
        
        # If running is False and the competition script is running, kill it.
        elif not running and main_process is not None:
            oled_display.reset()
            oled_display.text("EXITING", 0, 0, size=10)
            motors.run(0, 0)
            
            print("Terminating competition_main.py...")
            subprocess.call(["sudo", "kill", "-9", str(main_process.pid)])
            main_process = None

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Exiting...")
    if main_process is not None:
        subprocess.call(["sudo", "kill", "-9", str(main_process.pid)])
        
finally:
    GPIO.cleanup()
