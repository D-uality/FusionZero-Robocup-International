import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(13, GPIO.OUT)

try:
    while True:
        GPIO.output(13, GPIO.HIGH)
        time.sleep(1)
        # GPIO.output(13, GPIO.LOW)
        # time.sleep(1)

except KeyboardInterrupt:
    print("Exiting Gracefully")

finally:
    GPIO.cleanup()