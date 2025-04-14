import cv2
import numpy as np

import motors
import line
import sensors

mode = 's'

dummy_image = 255 * np.ones((500, 500, 3), dtype=np.uint8)
cv2.imshow("Control Window", dummy_image)

try:
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
            sensors.read_mapped_analog(True)
        elif mode == 'c':
            sensors.calibration(True)
            mode = 's'

except KeyboardInterrupt:
    print("Exiting...")

finally:
    motors.run(0, 0)
    cv2.destroyAllWindows()
