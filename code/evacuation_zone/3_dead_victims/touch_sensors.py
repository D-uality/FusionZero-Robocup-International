import RPi.GPIO as GPIO
import oled_display
import config

def initialise() -> None:
    try:
        for touch_pin in config.touch_pins:
            GPIO.setup(touch_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        config.status_messages.append(["Touch", "✓"])
        oled_display.text("Touch: ✓", 0, 30)
        
    except Exception as e:
        config.status_messages.append(["Touch", "X", f"{e}"])
        oled_display.text("Touch: x", 0, 30)

def read(pins=config.touch_pins) -> list[int]:
    values = []

    for pin in pins:
        values.append(GPIO.input(pin))

    return values