import time

import gpio
import laser_sensors
import touch_sensors
import oled_display
import camera

gpio.initalise()
oled_display.initalise()
laser_sensors.initalise()
touch_sensors.initalise()
camera.initalise()

input("Press enter to begin program! ")
oled_display.reset()

try:
    while True:
        oled_display.text("Hello World", 0, 0)

except KeyboardInterrupt:
    print("Exiting Gracefully")

finally:
    gpio.cleanup()
    oled_display.reset()
