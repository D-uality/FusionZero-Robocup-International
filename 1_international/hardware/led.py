from core.shared_imports import GPIO, time

class LED():
    def __init__(self):
        self.pin = 13
        
        GPIO.setup(self.pin, GPIO.OUT)
        GPIO.output(self.pin, GPIO.LOW)
    
    def on(self):
        GPIO.output(self.pin, GPIO.HIGH)
        
    def off(self):
        GPIO.output(self.pin, GPIO.LOW)
        
    def blink(self, delay: float):
        self.off()
        time.sleep(delay)
        self.on()
        time.sleep(delay)