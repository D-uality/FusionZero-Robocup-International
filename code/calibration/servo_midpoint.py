import time
from adafruit_servokit import ServoKit

# BL TL TR BR
servo_pins = [14, 13, 12, 10]
pca = ServoKit(channels=16)
stop_angles = [89, 88, 89, 88]

try:
    while True:
        angle = int(input("angle to run at: "))

        for servo_pin in servo_pins:
            pca.servo[servo_pin].angle = angle

except KeyboardInterrupt:
    print("Exiting Gracefully")

finally:
    for pin_number, servo_pin in enumerate(servo_pins):
        pca.servo[servo_pin].angle = stop_angles[pin_number]