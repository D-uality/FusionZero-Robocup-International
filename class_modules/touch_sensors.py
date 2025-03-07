import RPi.GPIO as GPIO
import config

class c_touch_sensor():
    def __init__(self, touch_pin: int):
        self.__touch_pin = touch_pin
        GPIO.setup(self.__touch_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def read(self) -> int:
        return GPIO.input(self.__touch_pin)