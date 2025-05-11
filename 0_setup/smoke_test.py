#!/usr/bin/env python3
import importlib
import os
import traceback

modules = [
    "time", "socket", "typing", "operator", "os", "sys", "math",
    "cv2", "numpy", "multiprocessing", "threading", "random",
    "RPi.GPIO", "board",
    "adafruit_vl53l1x", "adafruit_servokit", "adafruit_ads7830",
    "busio", "adafruit_bno08x"
]

print("🧪 Running module import smoke test...\n")

for mod in modules:
    try:
        importlib.import_module(mod)
        print(f"✔️  {mod}")
    except Exception as e:
        print(f"❌  {mod}: {e}")

print("\n📷 Testing libcamera & Picamera2...")
try:
    import libcamera
    print(f"✔️  libcamera ({libcamera.__file__})")
except Exception as e:
    print(f"❌  libcamera: {e}")

try:
    from picamera2 import Picamera2
    print("✔️  Picamera2")
except Exception as e:
    print(f"❌  Picamera2: {e}")

print("\n🧠 Testing TFLite Runtime & Edge TPU delegate...")
try:
    from tflite_runtime.interpreter import Interpreter, load_delegate
    print("✔️  tflite_runtime interpreter imported")
    # Attempt to load the delegate
    for lib in ("libedgetpu.so.1",):
        try:
            delegate = load_delegate(lib)
            print(f"✔️  Edge TPU delegate loaded from '{lib}'")
            break
        except Exception:
            # try common library paths
            for d in ("/usr/lib", "/usr/lib/aarch64-linux-gnu", "/usr/lib/arm-linux-gnueabihf"):
                path = os.path.join(d, lib)
                if os.path.isfile(path):
                    try:
                        delegate = load_delegate(path)
                        print(f"✔️  Edge TPU delegate loaded from '{path}'")
                        raise StopIteration
                    except Exception as de2:
                        print(f"❌  delegate load failed for '{path}': {de2}")
            else:
                continue
    else:
        print("❌  Edge TPU delegate could not be loaded")
except Exception as e:
    print(f"❌  tflite_runtime or delegate import error: {e}")
