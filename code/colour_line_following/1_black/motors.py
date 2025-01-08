import time
from adafruit_servokit import ServoKit

servo_pins = [15, 14, 13, 12]
stop_angles = [97, 96, 96, 97]
pca = ServoKit(channels=16)

def run(v1: int, v2: int, delay: int = 0) -> None:
    calculated_angles = [0, 0, 0, 0]

    negative_gradients  = [0.80872, 0.87105, 0.68875, 0.79321]
    negative_intercepts = [-3.3078, -5.3841, -2.8756, -1.1818]
    positive_gradients  = [0.81624, 0.80004, 0.68003, 0.67527]
    positive_intercepts = [2.1236, 5.9884, 2.6411, 2.8736]
    
    for i in range(0, 2):
        if   (v1 < negative_intercepts[i]): calculated_angles[i] = negative_gradients[i] * v1 + negative_intercepts[i]
        elif (v1 > positive_intercepts[i]): calculated_angles[i] = positive_gradients[i] * v1 + positive_intercepts[i]
        else:                               calculated_angles[i] = 0

        calculated_angles[i] = max(min(calculated_angles[i], 90), -90)

    v2 = v2 * -1

    for i in range(2, 4):
        if   (v2 < negative_intercepts[i]): calculated_angles[i] = negative_gradients[i] * v2 + negative_intercepts[i]
        elif (v2 > positive_intercepts[i]): calculated_angles[i] = positive_gradients[i] * v2 + positive_intercepts[i]
        else:                               calculated_angles[i] = 0

        calculated_angles[i] = max(min(calculated_angles[i], 90), -90)

    pca.servo[servo_pins[0]].angle = int(max(min(stop_angles[0] + calculated_angles[0], 180), 0))
    pca.servo[servo_pins[1]].angle = int(max(min(stop_angles[1] + calculated_angles[1], 180), 0))
    pca.servo[servo_pins[2]].angle = int(max(min(stop_angles[2] + calculated_angles[2], 180), 0))
    pca.servo[servo_pins[3]].angle = int(max(min(stop_angles[3] + calculated_angles[3], 180), 0))

    if delay > 0:
        time.sleep(delay)

        pca.servo[servo_pins[0]].angle = stop_angles[0]
        pca.servo[servo_pins[1]].angle = stop_angles[1]
        pca.servo[servo_pins[2]].angle = stop_angles[2]
        pca.servo[servo_pins[3]].angle = stop_angles[3]
