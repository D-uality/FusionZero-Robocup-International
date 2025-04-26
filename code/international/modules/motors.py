import time
from adafruit_servokit import ServoKit
import operator
from utils import debug

pca = ServoKit(channels=16)

class cMOTORS():
    def __init__(self) -> None:
        self.servo_pins = [ 14, 13, 12, 10 ]
        self.claw_pin = 9
        self.claw_angle = 70
        pca.servo[self.claw_pin].actuation_range = 270

        # # Aidan
        # self.stop_angles = [ 97, 96, 96, 97 ]
        # self.negative_gradients = [1.17106, 1.15876, 0.50274, 0.53452]
        # self.negative_intercepts = [-3.8658, -1.5444, -2.8559, -2.4646]
        # self.positive_gradients = [0.87580, 0.82601, 1.78133, 1.81726]
        # self.positive_intercepts = [4.6227, 4.3606, 3.9117, 3.4639]

        # Frederick
        self.stop_angles = [ 88, 89, 88, 89 ]
        self.negative_gradients  = [0.70106, 0.70440, 0.66426, 0.69425]
        self.negative_intercepts = [-6.8834, -4.9707, -5.0095, -4.9465]
        self.positive_gradients  = [0.70924, 0.63035, 0.67858, 0.69916]
        self.positive_intercepts = [4.1509, 6.5548, 3.5948, 4.2287]
       
        pca.servo[self.servo_pins[0]].angle = self.stop_angles[0]
        pca.servo[self.servo_pins[1]].angle = self.stop_angles[1]
        pca.servo[self.servo_pins[2]].angle = self.stop_angles[2]
        pca.servo[self.servo_pins[3]].angle = self.stop_angles[3]

        pca.servo[self.claw_pin].angle = self.claw_angle
        debug(["INITIALISATION", "MOTORS", f"✓"], [24, 15, 50])
    
    def run(self, v1: float, v2: float, delay: float = 0) -> None:
        calculated_angles = [0, 0, 0, 0]

        for i in range(0, 2):
            if   (v1 < self.negative_intercepts[i]): calculated_angles[i] = self.negative_gradients[i] * v1 + self.negative_intercepts[i]
            elif (v1 > self.positive_intercepts[i]): calculated_angles[i] = self.positive_gradients[i] * v1 + self.positive_intercepts[i]
            else:                                   calculated_angles[i] = 0

            calculated_angles[i] = max(min(calculated_angles[i], 90), -90)

        v2 = v2 * -1

        for i in range(2, 4):
            if   (v2 < self.negative_intercepts[i]): calculated_angles[i] = self.negative_gradients[i] * v2 + self.negative_intercepts[i]
            elif (v2 > self.positive_intercepts[i]): calculated_angles[i] = self.positive_gradients[i] * v2 + self.positive_intercepts[i]
            else:                                   calculated_angles[i] = 0

            calculated_angles[i] = max(min(calculated_angles[i], 90), -90)
        try:
            pca.servo[self.servo_pins[0]].angle = max(min(self.stop_angles[0] + calculated_angles[0], 90+60), 90-40)
            pca.servo[self.servo_pins[1]].angle = max(min(self.stop_angles[1] + calculated_angles[1], 90+60), 90-40)
            pca.servo[self.servo_pins[2]].angle = max(min(self.stop_angles[2] + calculated_angles[2], 90+40), 90-40)
            pca.servo[self.servo_pins[3]].angle = max(min(self.stop_angles[3] + calculated_angles[3], 90+40), 90-40)

            if delay > 0:
                time.sleep(delay)
        except Exception as e:  
            print("I2C TIMEOUT!")
            debug(["INITIALISATION", "MOTORS", f"{e}"], [24, 15, 50])
            print()
            
            raise e
    def claw(self, angle: int) -> None:
        try:
            pca.servo[self.claw_pin].angle = angle
            self.claw_angle = angle
        except Exception as e:
            print("I2C TIMEOUT!")
            debug(["INITIALISATION", "MOTORS", f"{e}"], [24, 15, 50])
            print()
            
            raise e
            
    def claw_step(self, target_angle: int, time_delay: float) -> None:
        if target_angle == 270: target_angle = 269
        if target_angle == 0:   target_angle = 1
        current_angle = pca.servo[self.claw_pin].angle
        
        if current_angle == target_angle:
            return
        
        elif current_angle > target_angle:
            while current_angle > target_angle:
                current_angle -= 1
                pca.servo[self.claw_pin].angle = current_angle
                time.sleep(time_delay)
        
        elif current_angle < target_angle:
            while current_angle < target_angle:
                current_angle += 1
                pca.servo[self.claw_pin].angle = current_angle
                time.sleep(time_delay)
        else:
            pca.servo[self.claw_pin].angle = target_angle
            time.sleep(time_delay)

    def run_until(self, v1: float, v2: float, trigger_function: callable, index: int, comparison: str, target_value: int, text: str = "") -> None:
        if   comparison == "==": comparison_function = operator.eq
        elif comparison == "<=": comparison_function = operator.le
        elif comparison == ">=": comparison_function = operator.ge
        elif comparison == "!=": comparison_function = operator.ne

        while True:
            value = trigger_function()[index]
            if value is not None: break
            
        while True:
            value = trigger_function()[index]
            self.run(v1, v2)
            
            if value is not None: 
                debug([f"{text}", f"{value:.2f}", f"{target_value:.2f}"], [24, 10, 10])
                print()
                if comparison_function(value, target_value): break

        self.run(0, 0)

    def pause(self) -> None:
        self.run(0, 0)
        input()