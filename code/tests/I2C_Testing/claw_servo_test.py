import time
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)
kit.servo[11].actuation_range = 270

try:
    while True:
        key = input("Enter num for servo direction (0-270): ")

        # Check if input is a valid number
        num = int(key)

        kit.servo[11].angle = max(0, min(270, num))

        time.sleep(1)
except KeyboardInterrupt:
    print("Exiting program!")