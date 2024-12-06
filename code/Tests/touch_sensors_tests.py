import RPi.GPIO as GPIO
# Front Left, Front Right, Back Left, Back Right
touchPins =  5, 6, 22, 26 

GPIO.setmode(GPIO.BCM)
for touch_pin in touch_pins
    GPIO.setup(touch_pin, GPIO.IN)
    GPIO.setup(touch_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

try:
    while True:
        if GPIO.input(touch_pin) == GPIO.HIGH:
            print("1")
        else:
            print("0")

except KeyboardInterrupt:
    print("Exiting Gracefully")

finally:
    GPIO.cleanup()