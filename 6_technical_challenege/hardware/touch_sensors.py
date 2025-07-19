from core.shared_imports import GPIO

class TouchSensors():
    def __init__(self):
        self.touch_pins = [6, 26]

        for touch_pin in self.touch_pins:
            GPIO.setup(touch_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def read(self):
        values = [GPIO.input(pin) for pin in self.touch_pins]
        return values