#!/usr/bin/env bash
set -euo pipefail

# 1. Ensure uv is installed
if ! command -v uv &> /dev/null; then
  echo "Installing uv package manager…" 
  curl -LsSf https://astral.sh/uv/install.sh | sh                             
fi

# 2. Add Google Coral Edge TPU repository and key
sudo apt-get update
sudo apt-get install -y curl gnupg
if [ ! -f /usr/share/keyrings/coral-edgetpu-archive-keyring.gpg ]; then
  curl -fsSL https://packages.cloud.google.com/apt/doc/apt-key.gpg \
    | sudo tee /usr/share/keyrings/coral-edgetpu-archive-keyring.gpg >/dev/null
fi
if [ ! -f /etc/apt/sources.list.d/coral-edgetpu.list ]; then
  echo "deb [signed-by=/usr/share/keyrings/coral-edgetpu-archive-keyring.gpg] \
    https://packages.cloud.google.com/apt coral-edgetpu-stable main" \
    | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list
fi

# 3. Install system-level dependencies
sudo apt-get update
sudo apt-get install -y \
  python3.9-dev \        # Python 3.9 headers for building C extensions :contentReference[oaicite:0]{index=0}  
  libgpiod2 \            # GPIO library for Blinka (Adafruit) :contentReference[oaicite:1]{index=1}  
  libusb-1.0-0-dev \      # USB support for Edge TPU and sensors  
  libjpeg-dev \           # JPEG support for OpenCV  
  libatlas-base-dev \     # optimized math libraries for numpy  
  libv4l-dev \            # V4L2 headers for camera support  
  build-essential \       # gcc, make, etc.  
  rpicam-apps \           # renamed libcamera-apps on Bookworm :contentReference[oaicite:2]{index=2}  
  libedgetpu1-std         # Edge TPU runtime :contentReference[oaicite:3]{index=3}

# 4. Install all Python packages into Python 3.9 via uv
uv pip install \
  numpy \                        # core numerical library :contentReference[oaicite:4]{index=4}  
  opencv-python \                # cv2 bindings :contentReference[oaicite:5]{index=5}  
  RPi.GPIO \                     # legacy GPIO module :contentReference[oaicite:6]{index=6}  
  picamera2 \                    # libcamera-based Python API :contentReference[oaicite:7]{index=7}  
  adafruit-circuitpython-blinka \# board + busio abstraction   
  adafruit-circuitpython-vl53l1x \# ToF distance sensor  
  adafruit-circuitpython-servokit \# PWM servo driver  
  adafruit-circuitpython-ads7830 \ # ADS7830 ADC  
  adafruit-circuitpython-bno08x \ # BNO08x IMU  
  pycoral \                      # Coral Edge TPU Python library  
  tflite-runtime                 # TensorFlow Lite runtime  
  --python python3.9             # target the Python 3.9 interpreter :contentReference[oaicite:9]{index=9}
