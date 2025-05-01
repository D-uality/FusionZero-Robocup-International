import RPi.GPIO as GPIO
import config

def initialise() -> None:
    try:
        GPIO.setmode(GPIO.BCM)
        
        print(["GPIO", "✓"])
        
    except Exception as e:
        print(["GPIO", "X", f"{e}"])

def cleanup() -> None:
    GPIO.cleanup()