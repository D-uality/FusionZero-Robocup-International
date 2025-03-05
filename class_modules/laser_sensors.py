import RPi.GPIO as GPIO
import adafruit_vl53l1x
import board
import time
from typing import Optional

class c_laser_sensor():
    def __init__(self, x_shut_pin, new_i2c_address):
        i2c = board.I2C()
        
        GPIO.output(x_shut_pin, GPIO.HIGH)
        self.sensor = adafruit_vl53l1x.VL53L1X(i2c)
        self.sensor.set_address(new_i2c_address)

        self.sensor.start_ranging()

    def read(self) -> Optional[int]: 
        start_time = time.time()
        while not self.sensor.data_ready or self.sensor.distance is None:
            time.sleep(0.00001)
            if time.time() - start_time > 1: return None

        distance = self.sensor.distance
        self.sensor.clear_interrupt()

        return distance