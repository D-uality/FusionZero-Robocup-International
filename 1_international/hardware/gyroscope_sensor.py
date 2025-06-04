from core.shared_imports import math, mp, board, busio, BNO08X_I2C, BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE, BNO_REPORT_MAGNETOMETER, BNO_REPORT_ROTATION_VECTOR
from core.utilities import debug
import time

class Gyroscope():
    def __init__(self):
        self.__i2c = busio.I2C(board.SCL, board.SDA)
        self.__gyroscope = None
        self.__last_angles = None
        self.__unwrapped_angles = None
        self.__gyroscope = BNO08X_I2C(self.__i2c, address=0x4b)

        self.__gyroscope.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.__gyroscope.enable_feature(BNO_REPORT_GYROSCOPE)
        self.__gyroscope.enable_feature(BNO_REPORT_MAGNETOMETER)
        self.__gyroscope.enable_feature(BNO_REPORT_ROTATION_VECTOR)

        self.__data_queue      = mp.Queue()
        self.__exit_event      = mp.Event()
        self.__reading_process = mp.Process(target=self.__background_read)
        
        self.start()
        debug(["INITIALISATION", "GYROSCOPE", "✓"], [25, 25, 50])
    
    def start(self):
        self.__reading_process.start()
    
    def stop(self):
        self.__exit_event.set()
        
        if self.__reading_process.is_alive():
            self.__reading_process.terminate()
            self.__reading_process.join()

    def read(self):
        try:
            current_angles = self.__data_queue.get_nowait()
            
            # Initialize the unwrapping state on the first reading
            if self.__last_angles is None:
                self.__last_angles = current_angles[:]
                self.__unwrapped_angles = current_angles[:]
            else:
                # For each angle, calculate the delta and adjust for any wrapping jumps.
                for i in range(3):
                    delta = current_angles[i] - self.__last_angles[i]
                    if delta > 180: delta -= 360
                    elif delta < -180: delta += 360
                    self.__unwrapped_angles[i] += delta

                self.__last_angles = current_angles[:]

            return [int(angle) for angle in self.__unwrapped_angles]
        
        except Exception:
            return None
        
    def __background_read(self):
        try:
            while not self.__exit_event.is_set():
                try:
                    x, y, z, w = self.__gyroscope.quaternion

                    sinr_cosp = 2 * (w * x + y * z)
                    cosr_cosp = 1 - 2 * (x * x + y * y)
                    roll      = math.atan2(sinr_cosp, cosr_cosp)

                    sinp = 2 * (w * y - z * x)
                    if abs(sinp) >= 1: pitch = math.copysign(math.pi / 2, sinp)
                    else:              pitch = math.asin(sinp)

                    siny_cosp = 2 * (w * z + x * y)
                    cosy_cosp = 1 - 2 * (y * y + z * z)
                    yaw = math.atan2(siny_cosp, cosy_cosp)

                    roll_deg  = math.degrees(roll)
                    pitch_deg = math.degrees(pitch)
                    yaw_deg   = math.degrees(yaw)
                    current_angles = [roll_deg, pitch_deg, yaw_deg]

                    self.__data_queue.put(current_angles)
                    time.sleep(0.001)
                    
                except Exception as e:
                    print(f"Error reading gyroscope: {e}")
                    
        except KeyboardInterrupt:
            pass