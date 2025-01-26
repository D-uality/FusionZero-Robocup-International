import board
import adafruit_ssd1306
import time

i2c = board.I2C()
oled = adafruit_ssd1306.SSD1306_I2C(128, 64, i2c, addr=0x3c)

def reset():
    oled.fill(0)
    oled.show()