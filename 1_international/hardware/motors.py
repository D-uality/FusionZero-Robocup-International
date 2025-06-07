from core.shared_imports import time, ServoKit, socket, getpass, math, mp
from core.utilities import debug

class Motors():
    def __init__(self) -> None:
        self.pca = ServoKit(channels=16)
    
        username = getpass.getuser()
        hostname = socket.gethostname()
        self.user_at_host = f"{username}@{hostname}"
        
        if self.user_at_host == "frederick@raspberrypi":
            self.servo_pins = [14, 13, 12, 10]
            self.stop_angles = [97, 96, 96, 97]
            
            self.negative_gradients  = [-0.96727, -0.90880, -0.90704, -0.77515]
            self.negative_intercepts = [0.54149, 0.49693, 0.25736, 0.51568]
            self.positive_gradients  = [-1.00445, -0.93679, -0.72308, -0.78219]
            self.positive_intercepts = [-0.55059, -0.25260, -0.53597, -0.43908]

        elif self.user_at_host == "aidan@fusionzero":
            self.servo_pins = [14, 13, 12, 10]
            self.stop_angles = [88, 89, 88, 89]
            
            self.negative_gradients = [0.70106, 0.70440, 0.66426, 0.69425]
            self.negative_intercepts = [-6.8834, -4.9707, -5.0095, -4.9465]
            self.positive_gradients = [0.70924, 0.63035, 0.67858, 0.69916]
            self.positive_intercepts = [4.1509, 6.5548, 3.5948, 4.2287]
            
        else:
            print(self.user_at_host)
            raise ValueError(f"Unknown hostname: {hostname}")
       
        for i in range(0, 4):
            self.pca.servo[self.servo_pins[i]].angle = self.stop_angles[i]

        
        self.last_run = mp.Array('f', 2)   # [left, right] rad/s equiv.
        debug(["INITIALISATION", "MOTORS", "✓"], [25, 25, 50])
    
    def run(self, v1: float, v2: float, delay: float = 0) -> None:
        self.last_run[0] = v1
        self.last_run[1] = v2
        
        if self.user_at_host == "frederick@raspberrypi": v1, v2 = v2, v1
        calculated_angles = [0, 0, 0, 0]
        
        for i in range(0, 2):
            if   (v1 < self.negative_intercepts[i]): calculated_angles[i] = self.negative_gradients[i] * v1 + self.negative_intercepts[i]
            elif (v1 > self.positive_intercepts[i]): calculated_angles[i] = self.positive_gradients[i] * v1 + self.positive_intercepts[i]
            else:                                    calculated_angles[i] = 0

            calculated_angles[i] = max(min(calculated_angles[i], 90), -90)

        v2 = v2 * -1

        for i in range(2, 4):
            if   (v2 < self.negative_intercepts[i]): calculated_angles[i] = self.negative_gradients[i] * v2 + self.negative_intercepts[i]
            elif (v2 > self.positive_intercepts[i]): calculated_angles[i] = self.positive_gradients[i] * v2 + self.positive_intercepts[i]
            else:                                    calculated_angles[i] = 0

            calculated_angles[i] = max(min(calculated_angles[i], 90), -90)
        
        try:
            for i in range(0, 4): self.pca.servo[self.servo_pins[i]].angle = max(min(self.stop_angles[i] + calculated_angles[i], 90+60), 90-40)

            if delay > 0:
                time.sleep(delay)
                
        except Exception as e:
            debug(["I2C TIMEOUT", "MOTORS", f"{e}"], [25, 25, 50])
                    
    def steer(self, v: float, angle: float, delay: float = 0):
        """
        Steer the vehicle by adjusting left and right motor speeds based on a target angle.

        The "inner" wheel speed is linearly blended from +v at 0° steering (both wheels forward)
        to -v at ±90° (pivot in place), and clamped beyond that.
        """
        angle = max(min(angle, 90), -90)

        ratio = 1.0 - 2.0 * (abs(angle) / 90.0)
        ratio = max(-1.0, min(1.0, ratio))

        if angle >= 0: v1, v2 =         v, v * ratio
        else:          v1, v2 = v * ratio,         v

        self.run(v1, v2, delay)
        
    def ease(self, v: float, start_angle: float, end_angle: float, step: int, max_time: float):
        angles = list(range(start_angle, end_angle, step))
        N = len(angles)
        if N < 2: raise ValueError("Need at least two angles for easing profile.")

        # Calculate weights using a sine curve for ease-in, ease-out
        weights = [math.sin(math.pi * i / (N - 1)) for i in range(N)]
        total_weight = sum(weights)
        normalized_weights = [w / total_weight for w in weights]
        times = [w * max_time for w in normalized_weights]

        for angle, t in zip(angles, times):
            self.steer(v, angle, t)

    def pause(self) -> None:
        self.run(0, 0)
        input()