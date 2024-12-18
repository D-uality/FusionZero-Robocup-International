from config import *
import RPi.GPIO as GPIO

def initalise():
    try:
        GPIO.setmode(GPIO.BCM)
        print("GPIO Initalised as mode BCM")
    except Exception as e:
        pritn(f"Error initialising GPIO: {e}")

def cleanup():
    GPIO.cleanup()