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

def run(v1: float, v2: float, delay: float = 0) -> None:
    calculated_angles = [0, 0, 0, 0]

    negative_gradients  = [0.71106, 0.71440, 0.66426, 0.69425]
    negative_intercepts = [-6.8834, -4.9707, -5.0095, -4.9465]
    positive_gradients  = [0.71924, 0.64035, 0.67858, 0.69916]
    positive_intercepts = [4.1509, 6.5548, 3.5948, 4.2287]
    
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
    
    pca.servo[config.servo_pins[0]].angle = max(min(config.stop_angles[0] + calculated_angles[0], 150), 0)
    pca.servo[config.servo_pins[1]].angle = max(min(config.stop_angles[1] + calculated_angles[1], 150), 0)
    pca.servo[config.servo_pins[2]].angle = max(min(config.stop_angles[2] + calculated_angles[2], 150), 0)
    pca.servo[config.servo_pins[3]].angle = max(min(config.stop_angles[3] + calculated_angles[3], 150), 0)

    if delay > 0: time.sleep(delay)

def run_until(v1: float, v2: float, trigger_function: callable, index: int, comparison: str, target_value: int, text: str = "") -> None:
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

# def run_uphill(v1: int, v2: int, delay: int = 0) -> None:
#     calculated_angles = [0, 0, 0, 0]

#     negative_gradients  = [0.69106, 0.73440, 0.68426, 0.67425]
#     negative_intercepts = [-6.8834, -4.9707, -5.0095, -4.9465]
#     positive_gradients  = [0.69924, 0.66035, 0.69858, 0.67916]
#     positive_intercepts = [4.1509, 6.5548, 3.5948, 4.2287]

#     speeds = [0, 0, 0, 0]
#     speeds[0] = v2
#     speeds[1] = v1
#     speeds[2] = v1
#     speeds[3] = v2

#     for i in range(4):
#         current_speed = speeds[i]
#         if i in [2, 3]:
#             current_speed = -current_speed
#         if current_speed < negative_intercepts[i]:
#             calculated_angles[i] = negative_gradients[i] * current_speed + negative_intercepts[i]
#         elif current_speed > positive_intercepts[i]:
#             calculated_angles[i] = positive_gradients[i] * current_speed + positive_intercepts[i]
#         else:
#             calculated_angles[i] = 0
#         calculated_angles[i] = max(min(calculated_angles[i], 90), -90)
#     for i in range(4):
#         angle = config.stop_angles[i] + calculated_angles[i]
#         pca.servo[config.servo_pins[i]].angle = max(min(angle, 180), 0)
#     if delay > 0:
#         time.sleep(delay)

def pause() -> None:
    run(0, 0)
    input()
    
# def run(v1: float, v2: float, delay: float = 0) -> None:
#     STEP_SIZE = 75
#     VALID_DIFFERENCE = 10
    
#     negative_gradients  = [0.71106, 0.71440, 0.66426, 0.69425]
#     negative_intercepts = [-6.8834, -4.9707, -5.0095, -4.9465]
#     positive_gradients  = [0.71924, 0.64035, 0.67858, 0.69916]
#     positive_intercepts = [4.1509, 6.5548, 3.5948, 4.2287]

#     values = [v1, v1, -v2, -v2]
#     calculated_angles = [0, 0, 0, 0]

#     for i, v in enumerate(values):
#         if v < negative_intercepts[i]:      calculated_angles[i] = negative_gradients[i] * v + negative_intercepts[i]
#         elif v > positive_intercepts[i]:    calculated_angles[i] = positive_gradients[i] * v + positive_intercepts[i]
#         else:                               calculated_angles[i] = 0

#         calculated_angles[i] = max(min(calculated_angles[i], 90), -90)

#     target_angles = [
#         max(min(config.stop_angles[i] + calculated_angles[i], 180), 0)
#         for i in range(len(config.servo_pins))
#     ]
    
#     all_servos_set = False
    
#     while not all_servos_set:
#         all_servos_set = True

#         for i, pin in enumerate(config.servo_pins):
#             current_angle = pca.servo[pin].angle if pca.servo[pin].angle is not None else 90
#             target_angle = target_angles[i]
#             diff = target_angle - current_angle
            
#             # print(f"Servo[{i}] is currently at {current_angle}, trying to set to {target_angle}")

#             if abs(diff) <= VALID_DIFFERENCE: continue
#             elif abs(diff) >= STEP_SIZE:
#                 step = STEP_SIZE if diff > 0 else -STEP_SIZE
                
#                 new_angle = current_angle + step
                
#                 pca.servo[pin].angle = new_angle
#                 all_servos_set = False
                
#                 # print(f"\tServo[{i}] moving from {current_angle} to {new_angle} (target: {target_angle})")
                
#             elif current_angle != target_angle:
#                 pca.servo[pin].angle = target_angle
#                 all_servos_set = False
#                 # print(f"\tServo[{i}] setting final angle to {target_angle}")
                
#         # print()
            
#         if not all_servos_set: time.sleep(0.0005)

#     if delay > 0: time.sleep(delay)