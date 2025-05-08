from core.shared_imports import math, time, board, busio, BNO08X_I2C, BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE, BNO_REPORT_MAGNETOMETER, BNO_REPORT_ROTATION_VECTOR
from core.utilities import debug

class Gyroscope():
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
        
        debug(["INITIALISATION", "GYROSCOPE", "✓"], [24, 24, 50])

        self.read()

    def read(self):
        current_time = time.time()

        if current_time - self.last_read > self.delay_time:
            self.last_read = current_time

            try:
                x, y, z, w = self.gyroscope.quaternion

                sinr_cosp = 2 * (w * x + y * z)
                cosr_cosp = 1 - 2 * (x * x + y * y)
                roll      = math.atan2(sinr_cosp, cosr_cosp)

                sinp = 2 * (w * y - z * x)
                if abs(sinp) >= 1: pitch = math.copysign(math.pi / 2, sinp)
                else:              pitch = math.asin(sinp)

                siny_cosp = 2 * (w * z + x * y)
                cosy_cosp = 1 - 2 * (y * y + z * z)
                yaw = math.atan2(siny_cosp, cosy_cosp)

                # Convert from radians to degrees (wrapped values)
                roll_deg  = math.degrees(roll)
                pitch_deg = math.degrees(pitch)
                yaw_deg   = math.degrees(yaw)
                current_angles = [roll_deg, pitch_deg, yaw_deg]

                # Initialize the unwrapping state on the first reading
                if 'last_angles' not in globals() or last_angles is None:
                    last_angles      = current_angles[:]    # store a copy of current wrapped angles
                    unwrapped_angles = current_angles[:]    # initial unwrapped angles same as wrapped
                    
                else:
                    # For each angle, calculate the delta and adjust for any wrapping jumps.
                    for i in range(3):
                        delta = current_angles[i] - last_angles[i]
                        # If a jump occurs (e.g., from 179 to -179, delta will be -358),
                        # adjust delta to represent the actual small change.
                        if   delta > 180: delta -= 360
                        elif delta < -180: delta += 360
                        
                        unwrapped_angles[i] += delta

                    last_angles = current_angles[:]  # update wrapped angles for next call

                return [int(angle) for angle in unwrapped_angles]
            
            except Exception as e:
                print(f"Error reading gyroscope: {e}")
                return [None, None, None]