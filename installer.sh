#!/usr/bin/env bash
set -e          # exit on first error
set -u          # treat unset vars as errors
set -o pipefail # catch err inside pipes

###############################################################################
# 0.  Prep – keep Pi up-to-date & enable I²C
###############################################################################
sudo apt update
sudo apt full-upgrade -y
sudo raspi-config nonint do_i2c 0

###############################################################################
# 1.  Core system packages (binary wheels via apt)
###############################################################################
sudo apt install -y \
    python3-venv \
    python3-numpy \
    python3-opencv \
    python3-picamera2 \
    python3-rpi.gpio \
    libatlas-base-dev \
    git curl gnupg

###############################################################################
# 2.  Google Coral Edge-TPU runtime
###############################################################################
echo "deb [signed-by=/usr/share/keyrings/coral-edgetpu.gpg] \
https://packages.cloud.google.com/apt coral-edgetpu-stable main" | \
  sudo tee /etc/apt/sources.list.d/coral-edgetpu.list
curl -fsSL https://packages.cloud.google.com/apt/doc/apt-key.gpg | \
  sudo gpg --dearmor -o /usr/share/keyrings/coral-edgetpu.gpg
sudo apt update
sudo apt install -y libedgetpu1-std   # or libedgetpu1-max if you overclock the TPU

###############################################################################
# 3.  Python venv that *inherits* apt-installed packages
###############################################################################
APP_DIR=/home/frederick/FusionZero-Robocup-International
python3 -m venv "$APP_DIR/venv" --system-site-packages
source "$APP_DIR/venv/bin/activate"

###############################################################################
# 4.  pip packages
###############################################################################
python -m pip install --upgrade pip wheel

# Coral + TF-Lite
python -m pip install --extra-index-url https://google-coral.github.io/py-repo/ \
    pycoral tflite-runtime

# Adafruit sensor drivers (+ Blinka gives you `board`, `busio`, etc.)
python -m pip install \
    adafruit-circuitpython-ads7830 \
    adafruit-circuitpython-vl53l1x \
    adafruit-circuitpython-bno08x \
    adafruit-circuitpython-servokit \
    adafruit-blinka

###############################################################################
# 5.  Smoke-test
###############################################################################
python - <<'PY'
import cv2, numpy, board, tflite_runtime.interpreter as tflite
print("OpenCV   :", cv2.__version__)
print("NumPy    :", numpy.__version__)
print("Board OK :", hasattr(board, "SCL"))
print("TFLite   :", tflite.__file__)
PY

echo
echo "✅  All set!  Activate any time with:"
echo "source $APP_DIR/venv/bin/activate"
