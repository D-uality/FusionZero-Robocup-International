from gpiozero import LED
from time import sleep

led = LED(13)

while True:
    led.on()
    print("LED is ON")
    sleep(2)
    # led.off()
    print("LED is OFF")
    sleep(2)


