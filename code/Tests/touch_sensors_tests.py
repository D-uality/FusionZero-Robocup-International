import RPi.GPIO as GPIO

touchPin = 15

GPIO.setmode(GPIO.BCM)
GPIO.setup(touchPin, GPIO.IN)
GPIO.setup(touchPin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

try:
    while True:
        if GPIO.input(touchPin) == GPIO.HIGH:
            print("1")
        else:
            print("0")

except KeyboardInterrupt:
    print("Exiting Gracefully")

finally:
    GPIO.cleanup()