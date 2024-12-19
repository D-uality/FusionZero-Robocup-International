from config import *
import time
from adafruit_servokit import ServoKit
import operator

pca = ServoKit(channels=16)

def run(v1, v2, delay=0):

    calculatedAngles = [0, 0, 0, 0]

    negativeGradients  = [0.64106, 0.68440, 0.68426, 0.67425]
    negativeIntercepts = [-6.8834, -4.9707, -5.0095, -4.9465]
    positiveGradients  = [0.64924, 0.61035, 0.69858, 0.67916]
    positiveIntercepts = [4.1509, 6.5548, 3.5948, 4.2287]
    
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

    pca.servo[servo_pins[0]].angle = stop_angles[0] + calculatedAngles[0]
    pca.servo[servo_pins[1]].angle = stop_angles[1] + calculatedAngles[1]
    pca.servo[servo_pins[2]].angle = stop_angles[2] + calculatedAngles[2]
    pca.servo[servo_pins[3]].angle = stop_angles[3] + calculatedAngles[3]

    if delay > 0:
        time.sleep(delay)

        pca.servo[servo_pins[0]].angle = stop_angles[0]
        pca.servo[servo_pins[1]].angle = stop_angles[1]
        pca.servo[servo_pins[2]].angle = stop_angles[2]
        pca.servo[servo_pins[3]].angle = stop_angles[3]

def run_until(v1, v2, trigger_function, index, comparison, target_value):
    if   comparison == "==": comparison_function = operator.eq
    elif comparison == "<=": comparison_function = operator.le
    elif comparison == ">=": comparison_function = operator.ge
    elif comparison == "!=": comparison_function = operator.ne

    while not comparison_function(trigger_function()[index], target_value):
        run(v1, v2)
        print()