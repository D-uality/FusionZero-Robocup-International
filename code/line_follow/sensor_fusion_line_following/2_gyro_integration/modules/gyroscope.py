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
delay_time, last_read = 0.08, 0

def initialise():
    global gyroscope
    try:
        gyroscope = BNO08X_I2C(i2c, address=0x4b)

        gyroscope.enable_feature(BNO_REPORT_ACCELEROMETER)
        gyroscope.enable_feature(BNO_REPORT_GYROSCOPE)
        gyroscope.enable_feature(BNO_REPORT_MAGNETOMETER)
        gyroscope.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        
        config.update_log(["INITIALISATION", "GYROSCOPE", "✓"], [24, 24, 3])
    except Exception as e:
        config.update_log(["INITIALISATION", "GYROSCOPE", "X"], [24, 24, 3])
        print(f"Failed to initialise gyroscope: {e}")
        exit()

def read():
    global gyroscope
    global last_read, delay_time
    
    current_time = time.time()
    
    if current_time - last_read > delay_time:
        last_read = current_time
        
        x, y, z, w = gyroscope.quaternion
        
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1: pitch = math.copysign(math.pi / 2, sinp)
        else:              pitch = math.asin(sinp)
        
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return [roll * 100, pitch * 100, yaw * 100]
    else:
        return [None, None, None]