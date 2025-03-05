import time
from adafruit_servokit import ServoKit
import operator
import config

class c_motors():
    def __init__(self, servo_pins: list[int], stop_angles: list[int]):
        self.pca = ServoKit(channels=16)
        
        self.__servo_pins  = servo_pins
        self.__stop_angles = stop_angles

        for pin in self.__servo_pins:
            self.pca.servo[pin].angle = self.__stop_angles[pin]

    def run(self, v1: float, v2: float, delay: float = 0) -> None:
        calculated_angles = [0, 0, 0, 0]

        gradients = [
            (0.71106, 0.71924), (0.71440, 0.64035),
            (0.66426, 0.67858), (0.69425, 0.69916)
        ]
        intercepts = [
            (-6.8834, 4.1509), (-4.9707, 6.5548),
            (-5.0095, 3.5948), (-4.9465, 4.2287)
        ]

        for i in range(4):
            v = v1 if i < 2 else v2 * -1
            neg_grad, pos_grad = gradients[i]
            neg_int,  pos_int  = intercepts[i]

            if v < neg_int:   calculated_angles[i] = neg_grad * v + neg_int
            elif v > pos_int: calculated_angles[i] = pos_grad * v + pos_int
            else:             calculated_angles[i] = 0

            calculated_angles[i] = max(min(calculated_angles[i], 90), -90)

        for i in range(4): self.pca.servo[self.__servo_pins[i]].angle = max(min(self.__stop_angles[i] + calculated_angles[i], 135), 45)
        if delay > 0: time.sleep(delay)

    def run_until(self, v1: float, v2: float, trigger_function: callable, index: int, comparison: str, target_value: int, text: str = "", max_time: float = 0) -> None:
        if   comparison == "==": comparison_function = operator.eq
        elif comparison == "<=": comparison_function = operator.le
        elif comparison == ">=": comparison_function = operator.ge
        elif comparison == "!=": comparison_function = operator.ne

        start_time = time.time()
        value = trigger_function()[index]
        while not comparison_function(value, target_value):
            if max_time > 0 and (time.time() - start_time) >= max_time: break

            self.run(v1, v2)
            value = trigger_function()[index]
            
            if text != "":
                config.update_log([text, value], [24, 10])

            time.sleep(0.01)

        self.run(0, 0)

    def pause(self) -> None:
        self.run(0, 0)
        input()

class c_claw():
    def __init__(self, claw_pin: int):
        self.pca = ServoKit(channels=16)
        self.__claw_pin = claw_pin

        self.pca.servo[self.__claw_pin].acutation_range = 270
        self.pca.servo[self.__claw_pin].angle = 270

    def step(self, target_angle: int, time_delay: float) -> None:
        if target_angle == 270: target_angle = 269
        if target_angle == 0:   target_angle = 1
        current_angle = self.pca.servo[self.__claw_pin].angle
        
        if current_angle == target_angle:
            return None
        
        elif current_angle > target_angle:
            while current_angle > target_angle:
                current_angle -= 1
                self.pca.servo[self.__claw_pin].angle = current_angle
                time.sleep(time_delay)
        else:
            while current_angle < target_angle:
                current_angle += 1
                self.pca.servo[self.__claw_pin].angle = current_angle
                time.sleep(time_delay)