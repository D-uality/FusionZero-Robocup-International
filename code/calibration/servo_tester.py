import time
from adafruit_servokit import ServoKit

servos = ServoKit(channels=16)
servo_pins = [14, 13, 12, 10]
# stop_angles = [89, 88, 89, 88]
stop_angles = [97, 96, 96, 97]

def run(v1, v2, delay=0):

    calculatedAngles = [0, 0, 0, 0]

    negativeGradients  = [0.84913, 0.81844, 0.65875, 0.66282]
    negativeIntercepts = [-4.9127, -1.4555, -2.5299, -3.2026]
    positiveGradients  = [0.82710, 0.78876, 0.64755, 0.65793]
    positiveIntercepts = [4.2989, 3.5638, 2.6239, 2.5256]

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