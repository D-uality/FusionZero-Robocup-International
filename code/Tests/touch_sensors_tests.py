import RPi.GPIO as GPIO
import time

buttonPin = 22

GPIO.setmode(GPIO.BCM)
GPIO.setup(buttonPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

try:
    while True:
        if GPIO.input(buttonPin) == GPIO.LOW:
            print("1")
        else:
            print("0")
        time.sleep(0.01)

except KeyboardInterrupt:
    print("Exiting Gracefully")

finally:
    GPIO.cleanup()