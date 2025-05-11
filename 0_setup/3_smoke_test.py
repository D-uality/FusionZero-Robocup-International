#!/usr/bin/env python3
import sys

def test_import(name, alias=None):
    try:
        if alias:
            __import__(alias)
        else:
            __import__(name)
        print(f"OK: {name}")
    except Exception as e:
        print(f"ERROR: {name} -> {e}")

if __name__ == '__main__':
    print("Starting smoke test imports...")
    modules = [
        'cv2',
        'numpy',
        ('Picamera2', 'picamera2'),
        'libcamera',
        'board',
        'busio',
        'adafruit_vl53l1x',
        'adafruit_servokit',
        'adafruit_ads7830',
        'adafruit_bno08x',
        ('tflite_runtime.interpreter', 'tflite_runtime.interpreter')
    ]
    for m in modules:
        if isinstance(m, tuple):
            test_import(m[0], m[1])
        else:
            test_import(m)
    print("Smoke test complete.")