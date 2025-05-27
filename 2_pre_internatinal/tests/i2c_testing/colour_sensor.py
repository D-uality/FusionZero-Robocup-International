# SPDX-FileCopyrightText: 2023 Liz Clark for Adafruit Industries
# SPDX-License-Identifier: MIT

# Simple demo to read analog input on all channels

import time
import board
import adafruit_ads7830.ads7830 as ADC
from adafruit_ads7830.analog_in import AnalogIn

# Initialize I2C and ADS7830
i2c = board.I2C()
adc = ADC.ADS7830(i2c)

# Function to read all channels
def read_all_channels():
    analog_readings = []
    for channel in range(6, 8):
        analog_readings.append(int(AnalogIn(adc, channel).value / 256))
        time.sleep(0.001)

    return analog_readings
    
while True:
    # Read all channels
    adc_values = read_all_channels()
    
    # Format and print all values on one line
    formatted_output = " | ".join([f"ADC{channel}: {value}" for channel, value in enumerate(adc_values)])
    print(formatted_output)
