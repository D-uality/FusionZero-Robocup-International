#!/usr/bin/env bash
set -euo pipefail

echo
echo "=== 1) Python interpreter & version ==="
which python3
python3 --version

echo
echo "=== 2) venv site-packages & sys.path ==="
python3 - <<'PY'
import sysconfig, pprint, sys
print(" purelib:", sysconfig.get_path("purelib"))
print(" platlib:", sysconfig.get_path("platlib"))
print("\n sys.path:")
pprint.pprint(sys.path)
PY

echo
echo "=== 3) Locate the libcamera package on disk ==="
echo "Listing /usr/lib/python3/dist-packages/libcamera:"
ls -l /usr/lib/python3/dist-packages/libcamera || echo "  (dir not found)"

echo
echo "=== 4) Module spec for libcamera ==="
python3 - <<'PY'
import importlib.util
spec = importlib.util.find_spec("libcamera")
print("libcamera spec:", spec)
if spec and spec.origin:
    print(" origin:", spec.origin)
PY

echo
echo "=== 5) Peek at libcamera/__init__.py ==="
head -n 20 /usr/lib/python3/dist-packages/libcamera/__init__.py || echo "  (cannot open __init__.py)"

echo
echo "=== 6) Try importing libcamera directly ==="
python3 - <<'PY'
import importlib, traceback
for m in ["libcamera", "libcamera._libcamera"]:
    try:
        importlib.import_module(m)
        print(f"✔️  imported {m}")
    except Exception as e:
        print(f"❌  failed {m}:")
        traceback.print_exc()
PY

echo
echo "=== 7) Inspect picamera2 import error ==="
python3 - <<'PY'
import traceback
try:
    import picamera2
    print("✔️  picamera2 imported")
except Exception as e:
    print("❌  picamera2 import raised:")
    traceback.print_exc()
PY

echo
echo "=== 8) Check PyCoral internals too ==="
python3 - <<'PY'
import pycoral, os
print("PyCoral path:", pycoral.__path__[0])
print("Contents:", os.listdir(pycoral.__path__[0] + "/utils"))
try:
    import pycoral.utils.edgetpu
    print("✔️  pycoral.utils.edgetpu imported")
except Exception as e:
    print("❌  pycoral.utils.edgetpu import raised:", e)
PY

echo
echo "=== Debug complete. Share any ❌ lines above ==="
