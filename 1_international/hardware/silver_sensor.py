from core.shared_imports import os, time, board, ADC, AnalogIn, socket, getpass
from core.utilities import debug

class SilverSensor():
    def __init__(self):
        self.__i2c = board.I2C()
        self.__ADC = ADC.ADS7830(self.__i2c)
        username = getpass.getuser()
        hostname = socket.gethostname()
        self.user_at_host = f"{username}@{hostname}"
        
        if self.user_at_host == "frederick@raspberrypi":
            self.sensor_pin = 5
        else:
           self.sensor_pin = 5

        debug( ["INITIALISATION", "SILVER SENSOR"], [25, 25] )
            
    def read(self) -> list[int]:
        value = int(AnalogIn(self.__ADC, self.sensor_pin).value / 256)
        return value