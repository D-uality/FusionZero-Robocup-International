import time
start_time = time.time()
import cv2
import numpy as np
from random import randint

from config import *
import gpio
import laser_sensors
import touch_sensors
import oled_display
import camera
import motors

import live_victims

try:
    gpio.initialise()
    oled_display.initialise()
    laser_sensors.initialise()
    touch_sensors.initialise()
    camera.initialise()

    input(f"({time.time() - start_time:.2f}) Press enter to begin program! ")
    oled_display.reset()

    display_found = False
    display_reset = False

    while True:
        start_time = time.time()
        image = camera.capture_array()

        motors.run(25, 25)

        touch_values = touch_sensors.read([touch_pins[0], touch_pins[1]])
        if touch_values[0] == 0 or touch_values[1] == 0:
            direction = 2 * randint(0, 1) - 1
            motors.run(25 * direction, -25 * direction, randint(800, 1600) / 1000)
        
        live_x, live_y = live_victims.find(image, 7)

        if live_x is not None and live_y is not None:
            if not display_found:
                oled_display.text("FOUND", 0, 0, size=20)
                display_found = True
                display_reset = False

            distance = laser_sensors.read([x_shut_pins[1]])
            while distance > 15:
                live_victims.route(v=25, kP=1)

            motors.run(0, 0)

        else:
            if not display_reset:
                oled_display.reset()
                display_found = False
                display_reset = True

        if X11:
            cv2.imshow("image", image)

        FPS = (1)/(time.time() - start_time)
        print(f"{FPS=:.2f}")

except KeyboardInterrupt:
    print("Exiting Gracefully")

finally:
    gpio.cleanup()
    oled_display.reset()
    camera.close()
