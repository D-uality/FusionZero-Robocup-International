import cv2
import numpy as np
import time

import motors
import line
import colour
import laser_sensors
import oled_display
import touch_sensors
import camera

mode = 's'

dummy_image = 255 * np.ones((300, 300, 3), dtype=np.uint8)
cv2.imshow("Control Window", dummy_image)

try:
    oled_display.initialise()
    laser_sensors.initialise()
    touch_sensors.initialise()
    motors.initialise()
    camera.initialise()
    
    oled_display.reset()
    
    while True:
        key = cv2.waitKey(1) & 0xFF

        if key != -1:
            if key >= 32 and key <= 126:  # Check if key is an ASCII character
                mode = chr(key)
                print(f"Recieved input: {mode}")

        if mode == 's':
            motors.run(0, 0)
        elif mode == 'g':
            line.follow_line()
        elif mode == 'r':
            colour.read(display_mapped=True, display_raw=True)
            laser_sensors.read(display=True)
            touch_sensors.read(display=True)
            print()
        elif mode == 'c':
            colour.calibration(True)
            mode = 's'

except KeyboardInterrupt:
    print("Exiting...")

finally:
    motors.run(0, 0)
    cv2.destroyAllWindows()
    camera.close()
    motors.claw_step(270, 0)
