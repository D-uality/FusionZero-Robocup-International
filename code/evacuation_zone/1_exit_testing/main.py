import time
import cv2
import numpy

from motor import *

servos = ServoKit(channels=16)
servoPins = [14, 13, 12, 10]

def mapValue(x, inMin, inMax, outMin, outMax):
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin

try:
    while True:
        run(30, 30)

except KeyboardInterrupt:
    print("Exiting Gracefully")

finally:
    run(0, 0)