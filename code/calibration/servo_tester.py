import time
from adafruit_servokit import ServoKit

servos = ServoKit(channels=16)
servo_pins = [14, 13, 12, 10]
stop_angles = [89, 88, 89, 88]
# servo_pins = [15, 14, 13, 12]
# stop_angles = [97, 96, 96, 97]

def run(v1, v2, delay=0):

    calculatedAngles = [0, 0, 0, 0]

    negativeGradients  = [0.69106, 0.73440, 0.68426, 0.67425]
    negativeIntercepts = [-6.8834, -4.9707, -5.0095, -4.9465]
    positiveGradients  = [0.69924, 0.66035, 0.69858, 0.67916]
    positiveIntercepts = [4.1509, 6.5548, 3.5948, 4.2287]

    # negativeGradients  = [0.80872, 0.87105, 0.68875, 0.79321]
    # negativeIntercepts = [-3.3078, -5.3841, -2.8756, -1.1818]
    # positiveGradients  = [0.81624, 0.80004, 0.68003, 0.67527]
    # positiveIntercepts = [2.1236, 5.9884, 2.6411, 2.8736]
    
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
    
    for i in range(4):
        print(f"Servo {i}: stop_angle = {stop_angles[i]}, calculatedAngle = {calculatedAngles[i]}, total = {stop_angles[i] + calculatedAngles[i]}")

    servos.servo[servo_pins[0]].angle = stop_angles[0] + calculatedAngles[0]
    servos.servo[servo_pins[1]].angle = stop_angles[1] + calculatedAngles[1]
    servos.servo[servo_pins[2]].angle = stop_angles[2] + calculatedAngles[2]
    servos.servo[servo_pins[3]].angle = stop_angles[3] + calculatedAngles[3]

try:
    while True:
        v1, v2 = list(map(int, input("Angle (v1, v2): ").split(" ")))
        run(v1, v2)

except KeyboardInterrupt:
    print("Exiting Gracefully")

finally:
    run(0, 0)