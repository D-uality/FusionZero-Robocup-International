import time
from adafruit_servokit import ServoKit

servos = ServoKit(channels=16)

servoPins = [14, 13, 12, 10]

def mapValue(x, inMin, inMax, outMin, outMax):
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin

def run(v1, v2, delay=0):
    v1 = mapValue(v1, -100, 100, 0, 180)
    v2 = mapValue(v2, -100, 100, 180, 0)
    v1 = max(min(v1, 180), 0)
    v2 = max(min(v2, 180), 0)

    print(v1, v2)

    servos.servo[servoPins[0]].angle = v1
    servos.servo[servoPins[1]].angle = v1
    servos.servo[servoPins[2]].angle = v2
    servos.servo[servoPins[3]].angle = v2

try:
    while True:
        run(20, 20)

except KeyboardInterrupt:
    print("Exiting Gracefully")

finally:
    for pin in servoPins:
        servos.servo[pin].angle = 90