import time
from adafruit_servokit import ServoKit

servos = ServoKit(channels=16)
servoPins = [14, 13, 12, 10]

def mapValue(x, inMin, inMax, outMin, outMax):
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin

def run(v1, v2, delay=0):

    calculatedAngles = [0, 0, 0, 0]

    negativeGradients  = [0.64428, 0.63716, 0.67055, 0.65467]
    negativeIntercepts = [-7.2922, -6.9004, -6.9216, -7.7659]
    positiveGradients  = [0.65340, 0.66061, 0.69259, 0.69624]
    positiveIntercepts = [1.9929, 1.7728, 2.9945, 2.0946]
    
    for i in range(0, 2):
        if   (v1 < negativeIntercepts[i]): calculatedAngles[i] = negativeGradients[i] * v1 + negativeIntercepts[i];
        elif (v1 > positiveIntercepts[i]): calculatedAngles[i] = positiveGradients[i] * v1 + positiveIntercepts[i];
        else:                              calculatedAngles[i] = 0;

        calculatedAngles[i] = max(min(calculatedAngles[i], 90), -90)

    v2 = v2 * -1

    for i in range(2, 4):
        if   (v2 < negativeIntercepts[i]): calculatedAngles[i] = negativeGradients[i] * v2 + negativeIntercepts[i];
        elif (v2 > positiveIntercepts[i]): calculatedAngles[i] = positiveGradients[i] * v2 + positiveIntercepts[i];
        else:                              calculatedAngles[i] = 0;

        calculatedAngles[i] = max(min(calculatedAngles[i], 90), -90)

    servos.servo[servoPins[0]].angle = 90 + calculatedAngles[0]
    servos.servo[servoPins[1]].angle = 90 + calculatedAngles[1]
    servos.servo[servoPins[2]].angle = 90 + calculatedAngles[2]
    servos.servo[servoPins[3]].angle = 90 + calculatedAngles[3]

try:
    while True:
        run(10, 10)

except KeyboardInterrupt:
    print("Exiting Gracefully")

finally:
    run(0, 0)