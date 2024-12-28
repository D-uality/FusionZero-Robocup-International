import RPi.GPIO as GPIO
import oled_display
import logging
import config

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def initialise() -> None:
    try:
        for touch_pin in config.touch_pins:
            GPIO.setup(touch_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        logging.info("Touch initialised!")
        oled_display.text("Touch: ✓", 0, 30)
        
    except Exception as e:
        logging.error(f"Touch failed to initialise: {e}")
        oled_display.text("Touch: x", 0, 30)

def read(pins=config.touch_pins) -> list[int]:
    values = []

    for pin in pins:
        values.append(GPIO.input(pin))

    logging.info(f"Touch: {values}")
    return values