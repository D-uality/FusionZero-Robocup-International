import RPi.GPIO as GPIO
import logging

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def initialise() -> None:
    try:
        GPIO.setmode(GPIO.BCM)
        logging.info("GPIO initialised!")
        
    except Exception as e:
        logging.error(f"GPIO failed to initialise: {e}")

def cleanup() -> None:
    GPIO.cleanup()