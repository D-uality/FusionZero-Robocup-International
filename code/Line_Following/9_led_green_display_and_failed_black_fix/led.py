import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(13, GPIO.OUT)

def led_on():
    GPIO.output(13, GPIO.HIGH)

def led_off():
    GPIO.output(13, GPIO.LOW)

def led_blink(delay):
    led_off()
    time.sleep(delay)
    led_on()
    time.sleep(delay)

def led_close():
    GPIO.cleanup()