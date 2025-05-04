import RPi.GPIO as GPIO

class cTOUCH_SENSORS:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        self.touch_pins = [6, 26]

        for touch_pin in self.touch_pins:
            GPIO.setup(touch_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def read(self):
        values = []
        for pin in self.touch_pins:
            values.append(GPIO.input(pin))
        return values

    def close(self):
        GPIO.cleanup()