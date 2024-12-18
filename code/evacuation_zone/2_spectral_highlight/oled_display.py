from config import *

import time
import adafruit_ssd1306
import board
from PIL import Image, ImageDraw, ImageFont

i2c = board.I2C()
oled = adafruit_ssd1306.SSD1306_I2C(SCREEN_WIDTH, SCREEN_HEIGHT, i2c, addr=0x3c)
image = Image.new("1", (SCREEN_WIDTH, SCREEN_HEIGHT))

def initalise():
    try:
        oled.fill(0)
        oled.show()

        print("OLED Initalised!")
    except Exception as E:
        print(f"OLED failed to initalise as {e}")

def reset():
    global image
    image = Image.new("1", (SCREEN_WIDTH, SCREEN_HEIGHT))
    oled.fill(0)
    oled.show()

def text(text, x, y, size=10):
    font = ImageFont.truetype("/home/fusion-zero/.local/share/fonts/jetbrains-mono/JetBrainsMono-Regular.ttf", size=size)

    draw = ImageDraw.Draw(image)

    draw.text((x, y), str(text), fill="white", font=font)

    oled.image(image)
    oled.show()