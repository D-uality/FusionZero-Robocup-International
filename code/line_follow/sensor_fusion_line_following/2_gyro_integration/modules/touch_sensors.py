import RPi.GPIO as GPIO
import oled_display
import config

def initialise() -> None:
    try:
        for touch_pin in config.touch_pins:
            GPIO.setup(touch_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        config.update_log(["INITIALISATION", "TOUCH", "✓"], [24, 15, 50])
        print()
        oled_display.text("Touch: ✓", 0, 30)
        
    except Exception as e:
        config.update_log(["INITIALISATION", "TOUCH", f"{e}"], [24, 15, 50])
        print()
        oled_display.text("Touch: x", 0, 30)
        
        raise e

def read(pins=config.touch_pins, display=False) -> list[int]:
    values = []

    for pin in pins:
        values.append(GPIO.input(pin))

    if display:
        print(f"T: {values}", end=", ")

    return values