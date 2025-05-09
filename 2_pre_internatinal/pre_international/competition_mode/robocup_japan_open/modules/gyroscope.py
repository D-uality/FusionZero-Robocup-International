import math
import time

import board
import busio
import config
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)

i2c = busio.I2C(board.SCL, board.SDA)
gyroscope = None
delay_time, last_read = 0.02, 0
last_angles = unwrapped_angles = None
    
def initialise():
    global gyroscope
    try:
        gyroscope = BNO08X_I2C(i2c, address=0x4b)

        gyroscope.enable_feature(BNO_REPORT_ACCELEROMETER)
        gyroscope.enable_feature(BNO_REPORT_GYROSCOPE)
        gyroscope.enable_feature(BNO_REPORT_MAGNETOMETER)
        gyroscope.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        
        config.update_log(["INITIALISATION", "GYROSCOPE", "✓"], [24, 15, 50])
        print()

        read()
    except Exception as e:
        config.update_log(["INITIALISATION", "GYROSCOPE", f"{e}"], [24, 15, 50])
        print()
        
        raise e
        
        
def read():
    global gyroscope, last_read, delay_time, last_angles, unwrapped_angles

    current_time = time.time()

    if current_time - last_read > delay_time:
        last_read = current_time

        try:
            x, y, z, w = gyroscope.quaternion

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

            # Convert from radians to degrees (wrapped values)
            roll_deg = math.degrees(roll)
            pitch_deg = math.degrees(pitch)
            yaw_deg = math.degrees(yaw)
            current_angles = [roll_deg, pitch_deg, yaw_deg]

            # Initialize the unwrapping state on the first reading
            if 'last_angles' not in globals() or last_angles is None:
                last_angles = current_angles[:]       # store a copy of current wrapped angles
                unwrapped_angles = current_angles[:]    # initial unwrapped angles same as wrapped
            else:
                # For each angle, calculate the delta and adjust for any wrapping jumps.
                for i in range(3):
                    delta = current_angles[i] - last_angles[i]
                    # If a jump occurs (e.g., from 179 to -179, delta will be -358),
                    # adjust delta to represent the actual small change.
                    if delta > 180:
                        delta -= 360
                    elif delta < -180:
                        delta += 360
                    unwrapped_angles[i] += delta

                last_angles = current_angles[:]  # update wrapped angles for next call

            return unwrapped_angles
        except:
            return [None, None, None]
    else:
        return [None, None, None]
