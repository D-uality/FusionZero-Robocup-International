import math
import time

import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)

class cGYROSCOPE():
    def __init__(self):
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.gyroscope = None
        self.delay_time = 0.02
        self.last_read = 0
        self.last_angles = None
        self.unwrapped_angles = None

        self.gyroscope = BNO08X_I2C(self.i2c, address=0x4b)

        self.gyroscope.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.gyroscope.enable_feature(BNO_REPORT_GYROSCOPE)
        self.gyroscope.enable_feature(BNO_REPORT_MAGNETOMETER)
        self.gyroscope.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        
        print("INITIALISATION: GYROSCOPE ✓")
        print()

        self.read()

    def read(self):
        current_time = time.time()

        if current_time - self.last_read > self.delay_time:
            self.last_read = current_time

            try:
                x, y, z, w = self.gyroscope.quaternion

                sinr_cosp = 2 * (w * x + y * z)
                cosr_cosp = 1 - 2 * (x * x + y * y)
                roll = math.atan2(sinr_cosp, cosr_cosp)

                sinp = 2 * (w * y - z * x)
                if abs(sinp) >= 1:
                    pitch = math.copysign(math.pi / 2, sinp)
                else:
                    pitch = math.asin(sinp)

                siny_cosp = 2 * (w * z + x * y)
                cosy_cosp = 1 - 2 * (y * y + z * z)
                yaw = math.atan2(siny_cosp, cosy_cosp)

                angles = [roll, pitch, yaw]
                
                if self.unwrapped_angles is None:
                    self.unwrapped_angles = angles
                    self.last_angles = angles
                    return angles

                for i in range(3):
                    delta_angle = angles[i] - self.last_angles[i]
                    if delta_angle > math.pi:
                        delta_angle -= 2.0 * math.pi
                    elif delta_angle < -math.pi:
                        delta_angle += 2.0 * math.pi
                    self.unwrapped_angles[i] += delta_angle

                self.last_angles = angles
                self.angles_deg = [math.degrees(angle) for angle in self.unwrapped_angles]

                return self.angles_deg

            except Exception as e:
                print(f"Error reading gyroscope: {e}")