from config import *
import RPi.GPIO as GPIO

def initialise():
    try:
        GPIO.setmode(GPIO.BCM)
        print("GPIO initialised!")
        
    except Exception as e:
        print(f"GPIO failed to initialise: {e}")

def cleanup():
    GPIO.cleanup()