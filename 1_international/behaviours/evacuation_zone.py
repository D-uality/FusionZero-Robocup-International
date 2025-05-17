from core.shared_imports import time
from hardware.robot import *
from core.utilities import *

start_display()

class WallFollow:
    OFFSET = 3

    def __init__(self, kP: float, base_speed: int):
        self.kP = kP
        self.base_speed = base_speed

    def check_touch(self) -> None:
        touch_values = touch_sensors.read()
        
        if sum(touch_values) != 2:
            motors.run(-self.base_speed, -self.base_speed, 0.3)
            motors.run(-self.base_speed,  self.base_speed, 1.3)

    def update(self) -> None:
        self.check_touch()
        
        distance = laser_sensors.read([0])[0]
        if distance is None: return None

        error = distance - self.OFFSET
        raw_turn = self.kP * error

        effective_turn = raw_turn * (1 + 7 / max(distance, self.OFFSET))

        max_turn = self.base_speed * 0.5
        effective_turn = min(max(effective_turn, -max_turn), max_turn)

        left_speed  = self.base_speed + effective_turn
        right_speed = self.base_speed - effective_turn

        motors.run(left_speed, right_speed)
        debug([f"{error:.2f}", f"{raw_turn:.2f}", f"{left_speed:.2f} {right_speed:.2f}"],[30, 20, 10])

def main() -> None:
    # wall_follower = WallFollow(kP=1.2, base_speed=30)
    
    # while True:
    #     wall_follower.update()

    motors.run(30, 30)