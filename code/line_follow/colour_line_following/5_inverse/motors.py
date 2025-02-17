import time
from adafruit_servokit import ServoKit
import operator
import config

pca = ServoKit(channels=16)
pca.servo[config.claw_pin].actuation_range = 270

def initialise() -> None:
    try:
        pca.servo[config.servo_pins[0]].angle = config.stop_angles[0]
        pca.servo[config.servo_pins[1]].angle = config.stop_angles[1]
        pca.servo[config.servo_pins[2]].angle = config.stop_angles[2]
        pca.servo[config.servo_pins[3]].angle = config.stop_angles[3]

        pca.servo[config.claw_pin].angle = 270
        print("Motors", "✓")
    except Exception as e:
        print("Motors", "X")
        print(f"Failed to initialise motors: {e}")

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

    pca.servo[config.servo_pins[0]].angle = max(min(config.stop_angles[0] + calculated_angles[0], 180), 0)
    pca.servo[config.servo_pins[1]].angle = max(min(config.stop_angles[1] + calculated_angles[1], 180), 0)
    pca.servo[config.servo_pins[2]].angle = max(min(config.stop_angles[2] + calculated_angles[2], 180), 0)
    pca.servo[config.servo_pins[3]].angle = max(min(config.stop_angles[3] + calculated_angles[3], 180), 0)

    if delay > 0:
        time.sleep(delay)

def run_until(v1: int, v2: int, trigger_function: callable, index: int, comparison: str, target_value: int, text: str = "") -> None:
    if   comparison == "==": comparison_function = operator.eq
    elif comparison == "<=": comparison_function = operator.le
    elif comparison == ">=": comparison_function = operator.ge
    elif comparison == "!=": comparison_function = operator.ne

    value = trigger_function()[index]
    while not comparison_function(value, target_value) and value is not None:
        value = trigger_function()[index]
        run(v1, v2)

        print(config.update_log([f"{text}", f"{value}", f"{target_value}"], [24, 10, 10]))

    run(0, 0)

def claw_step(target_angle: int, time_delay: float) -> None:
    if target_angle == 270: target_angle = 269
    if target_angle == 0:   target_angle = 1
    current_angle = pca.servo[config.claw_pin].angle
    
    if current_angle == target_angle:
        return
    
    elif current_angle > target_angle:
        while current_angle > target_angle:
            current_angle -= 1
            pca.servo[config.claw_pin].angle = current_angle
            time.sleep(time_delay)
    
    else:
        while current_angle < target_angle:
            current_angle += 1
            pca.servo[config.claw_pin].angle = current_angle
            time.sleep(time_delay)