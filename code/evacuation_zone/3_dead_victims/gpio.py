import RPi.GPIO as GPIO
import config

def initialise() -> None:
    try:
        GPIO.setmode(GPIO.BCM)
        
        config.status_messages.append(["GPIO", "✓"])
        
    except Exception as e:
        config.status_messages.append(["GPIO", "X", f"{e}"])

def cleanup() -> None:
    GPIO.cleanup()