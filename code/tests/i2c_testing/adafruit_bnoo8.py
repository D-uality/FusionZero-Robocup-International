# SPDX-FileCopyrightText: 2020 Bryan Siepert, written for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense
import time
import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

i2c = busio.I2C(board.SCL, board.SDA)
bno = BNO08X_I2C(i2c, address=0x4b)

bno.enable_feature(BNO_REPORT_ACCELEROMETER)
bno.enable_feature(BNO_REPORT_GYROSCOPE)
bno.enable_feature(BNO_REPORT_MAGNETOMETER)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

import math

def quaternion_to_euler(q):
    x, y, z, w = q  # q = (x, y, z, w)
    # Roll (rotation around x-axis)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (rotation around y-axis)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)
    
    # Yaw (rotation around z-axis)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw

while True:
    try:
        time.sleep(0.1)
        # print("Acceleration:")
        accel_x, accel_y, accel_z = bno.acceleration  # pylint:disable=no-member
        # print("X: %0.6f  Y: %0.6f Z: %0.6f  m/s^2" % (accel_x, accel_y, accel_z))
        # print("")

        # print("Gyro:")
        gyro_x, gyro_y, gyro_z = bno.gyro  # pylint:disable=no-member
        # print("X: %0.6f  Y: %0.6f Z: %0.6f rads/s" % (gyro_x, gyro_y, gyro_z))
        # print("")

        # print("Magnetometer:")
        mag_x, mag_y, mag_z = bno.magnetic  # pylint:disable=no-member
        # print("X: %0.6f  Y: %0.6f Z: %0.6f uT" % (mag_x, mag_y, mag_z))
        # print("")

        # print("Rotation Vector Quaternion:")
        quat_i, quat_j, quat_k, quat_real = bno.quaternion  # pylint:disable=no-member
        # print(
        #     "I: %0.6f  J: %0.6f K: %0.6f  Real: %0.6f" % (quat_i, quat_j, quat_k, quat_real)
        # )
        # print("")
        
        # print(f"ACCEL: {accel_x:.2f}, {accel_y:.2f}, {accel_z:.2f}    |    GYRO: {gyro_x:.2f} {gyro_y:.2f} {gyro_z:.2f}    |    MAG: {mag_x:.2f} {mag_y:.2f} {mag_z:.2f}    |    VEC: {quat_i:.2f} {quat_j:.2f} {quat_k:.2f} {quat_real:.2f}")
        # print(quat_i)
        quat = (quat_i, quat_j, quat_k, quat_real)
        roll, pitch, yaw = quaternion_to_euler(quat)
        print("Roll: {:.2f}, Pitch: {:.2f}, Yaw: {:.2f}".format(roll, pitch, yaw))
    
    except:
        pass

    finally:
        pass