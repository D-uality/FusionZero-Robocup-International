#!/usr/bin/env bash
set -e

echo "=== Update & upgrade system ==="
sudo apt-get update && sudo apt-get upgrade -y

echo "=== Install Python 3.9 & core packages ==="
sudo apt-get install -y \
  python3-pip \
  python3-numpy \
  python3-opencv \
  python3-picamera2 \
  libcamera-apps python3-libcamera \
  python3-smbus \
  i2c-tools \
  build-essential

echo "=== Upgrade pip & install TensorFlow Lite runtime ==="
sudo pip3 install --upgrade pip
sudo pip3 install tflite-runtime
sudo pip3 install --upgrade --force-reinstall "numpy<2"

echo "=== Add Coral Edge TPU APT repo & key ==="
echo "deb [signed-by=/usr/share/keyrings/libedgetpu.gpg] https://packages.cloud.google.com/apt coral-edgetpu-stable main" \
  | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list
curl -fsSL https://packages.cloud.google.com/apt/doc/apt-key.gpg \
  | sudo gpg --dearmor -o /usr/share/keyrings/libedgetpu.gpg

echo "=== Install Coral Edge TPU runtime (USB only) ==="
sudo apt-get update
sudo apt-get install -y --no-install-recommends libedgetpu1-std

echo "=== Configure udev rule & permissions ==="
echo 'SUBSYSTEM=="apex_0", GROUP="plugdev", MODE="0660"' \
  | sudo tee /etc/udev/rules.d/99-edgetpu.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG plugdev $USER

echo "✅ System packages and TPU runtime installed."