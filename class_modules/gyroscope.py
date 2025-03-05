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

class c_gyroscope():
    def __init__(self, delay_time: int):
        i2c = busio.I2C(board.SCL, board.SDA)
        
        self.gyroscope = BNO08X_I2C(i2c, address=0x4b)
        self.gyroscope.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.gyroscope.enable_feature(BNO_REPORT_GYROSCOPE)
        self.gyroscope.enable_feature(BNO_REPORT_MAGNETOMETER)
        self.gyroscope.enable_feature(BNO_REPORT_ROTATION_VECTOR)

        self.__delay_time = delay_time
        self.__last_read_time = 0
        self.__last_read_index = 0
        self.__record = [[None, None, None] for _ in range(10)]

    def __is_ready(self) -> bool:
        return True if time.time() - self.__last_read_time > self.__delay_time else False
    
    def __check(self) -> None:
        if all([all(x) for x in self.__record]): pass
        else:
            config.update(["FATAL ERROR", "Gyroscope stalling"], [config.method_size, 16])
            time.sleep(0.3)
            self.__init__(self.__delay_time)

    def read(self) -> tuple:
        roll, pitch, yaw = None, None, None

        if not self.__is_ready(): return roll, pitch, yaw
        
        self.__last_read_time = time.time()
        x, y, z, w = self.gyroscope.quaternion

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1: pitch = math.copysign(math.pi / 2, sinp)
        else:              pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.__record[self.__last_read_index] = [roll, pitch, yaw]
        self.__last_read_index = self.__last_read_index + 1 if self.__last_read_index < 9 else 0

        self.__check()
        
        return roll, pitch, yaw
    
    