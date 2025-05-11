#!/usr/bin/env bash
set -e

echo "=== Upgrade pip & install Adafruit Blinka & sensor libraries ==="
sudo pip3 install --upgrade pip
sudo pip3 install Adafruit-Blinka \
  adafruit-circuitpython-vl53l1x \
  adafruit-circuitpython-servokit \
  adafruit-circuitpython-ads7830 \
  adafruit-circuitpython-bno08x \
  adafruit-circuitpython-busdevice

echo "✅ Sensor libraries installed."