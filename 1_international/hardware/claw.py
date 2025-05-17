from core.shared_imports import time, ServoKit
from core.utilities import debug

class Claw():
    def __init__(self):
        self.lifter_pin = 9
        self.closer_pin = 8
        self.pca= ServoKit(channels=16)
        
        self.pca.servo[self.lifter_pin].angle = 160
        self.pca.servo[self.closer_pin].angle = 90
        
        debug(["INITIALISATION", "CLAW", "✓"], [24, 14, 50])
        
    def lift(self, target_angle: int, time_delay: float = 0) -> None:
        if target_angle < 20: target_angle = 20
        if target_angle > 160: target_angle = 160
        current_angle = self.pca.servo[self.lifter_pin].angle

        if current_angle == target_angle: return None

        if current_angle > target_angle:
            while current_angle > target_angle:
                current_angle -= 1
                self.pca.servo[self.lifter_pin].angle = current_angle
                time.sleep(time_delay)

        elif current_angle < target_angle:
            while current_angle < target_angle:
                current_angle += 1
                self.pca.servo[self.lifter_pin].angle = current_angle
                time.sleep(time_delay)

        else:
            self.pca.servo[self.lifter_pin].angle = target_angle
            time.sleep(time_delay)
        
    def close(self, angle: int) -> None:
        self.pca.servo[self.closer_pin].angle = angle
        