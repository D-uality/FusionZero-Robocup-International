import time
import os
import board
import adafruit_ads7830.ads7830 as ADC
from adafruit_ads7830.analog_in import AnalogIn

import motors

i2c = board.I2C()
adc = ADC.ADS7830(i2c)

CALIBRATION_FILE = "/home/fusion/FusionZero-Robocup-International/code/colour_line_following/calibration_values.txt"

def load_calibration_values():
    """
    Load calibration values from environment variables.
    
    return: Loads min calibrated values and max calibrated values from os.

    """
    if os.path.exists(CALIBRATION_FILE):
        with open(CALIBRATION_FILE, "r") as f:
            lines = f.readlines()
            min_values = list(map(int, lines[0].strip().split(",")))
            max_values = list(map(int, lines[1].strip().split(",")))

            print("Loaded calibrated min and max values:")
            print(f"Min: {min_values}")
            print(f"Max: {max_values}")
    else:
        print("Couldn't find saved calibrated values. Reseting colour values to default.")
        min_values = [255] * 7
        max_values = [0] * 7

    return min_values, max_values

calibrated_min, calibrated_max = load_calibration_values()

def read_raw(display=False):
    """
    Reads first 7 values from the ads7830 (colour sensor values).

    return: Array, size 7, raw adc readings from 0 to 255, with an order (0 to 7): Outer Left, Middle Left, Middle, Middle Right, Outer Right, Back Left, Back Right.
    """

    analog_readings = []
    for channel in range(7):
        analog_readings.append(int(AnalogIn(adc, channel).value / 256))
    
    if display:
        print(f"Raw C: {analog_readings}", end=", ")

    return analog_readings

def read(display_mapped=False, display_raw=False):
    """
    Reads raw analog data then maps values from 0 to 100 based on their min and max values

    return: mapped values as an array, size 7.
    """
    global calibrated_min, calibrated_max

    raw_analog_values = read_raw()

    mapped_values = []
    for i in range(7):
        mapped_value = (raw_analog_values[i] - calibrated_min[i]) * 100 / (calibrated_max[i] - calibrated_min[i])
        mapped_values.append(int(mapped_value))
    
    if display_mapped:
        print(f"C: {mapped_values}", end=", ")
    if display_raw:
        read_raw(True)

    return mapped_values

def update_calibration(min_values, max_values):
    for i in range(1000):
        analog_values = read_raw(True)
        print()

        for i in range(7):
            min_values[i] = min(min_values[i], analog_values[i])
            max_values[i] = max(max_values[i], analog_values[i])
        
        time.sleep(0.001)
    
    return min_values, max_values

def save_calibration_values(min_values, max_values):
    """
    Save calibration values to environment variables.
    """
    try:
        with open(CALIBRATION_FILE, "w") as f:
            f.write(",".join(map(str, min_values)) + "\n")
            f.write(",".join(map(str, max_values)) + "\n")
        print("Calibration values saved to file.")
    except IOError as e:
        print(f"Error saving calibration values: {e}")

def calibration(auto_calibrate):
    global calibrated_min, calibrated_max
    """
    Resets calibrated colour sensor values that are used for mapped values which is needed for line following.

    param auto_calibrate: If True it turns the motors automatically to callibrate without the need for a person to move the car, if False someone will move the car over the line to calibrate it.
    """
    
    calibrated_min = [255] * 7
    calibrated_max = [0] * 7

    if auto_calibrate:
        motors.run(30, -30)
    
    calibrated_min, calibrated_max = update_calibration(calibrated_min, calibrated_max)

    if auto_calibrate:
        motors.run(-30, 30)

    calibrated_min, calibrated_max = update_calibration(calibrated_min, calibrated_max)

    print(f"Min Values: {calibrated_min}")
    print(f"Max Values: {calibrated_max}")

    save_calibration_values(calibrated_min, calibrated_max)