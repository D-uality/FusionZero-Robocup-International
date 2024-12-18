from config import *
import RPi.GPIO as GPIO

def initalise():
    GPIO.setmode(GPIO.BCM)
    print("GPIO Initalised as mode BCM")

def cleanup():
    GPIO.cleanup()