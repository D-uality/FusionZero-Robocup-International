import time

start_time = time.perf_counter()

import socket
import getpass
from typing import Optional, Deque
from collections import deque
import operator
import os
import sys
import math
from queue import Empty

import cv2
import numpy as np
import multiprocessing as mp
from threading import Thread
from random import randint

from RPi import GPIO
from picamera2 import Picamera2
from libcamera import Transform

import board
import adafruit_vl53l1x
from adafruit_servokit import ServoKit
import adafruit_ads7830.ads7830 as ADC
from adafruit_ads7830.analog_in import AnalogIn
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)

GPIO.setmode(GPIO.BCM)

print(f"{time.perf_counter() - start_time:.2f}")