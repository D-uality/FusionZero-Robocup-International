import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(13, GPIO.OUT)

def on():
    GPIO.output(13, GPIO.HIGH)

def off():
    GPIO.output(13, GPIO.LOW)

def blink(delay):
    led_off()
    time.sleep(delay)
    led_on()
    time.sleep(delay)

def close():
    GPIO.cleanup()
