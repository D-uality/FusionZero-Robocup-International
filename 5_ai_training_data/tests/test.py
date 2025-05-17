#!/usr/bin/env python3
import pathlib, sys, time
import cv2, numpy as np
from tflite_runtime.interpreter import Interpreter, load_delegate
from pycoral.adapters import common   # pip install pycoral

# ─── CONFIG ────────────────────────────────────────────────────────────────────
MODEL_PATH  = pathlib.Path(
    "/home/frederick/FusionZero-Robocup-International/5_ai_training_data/models/dead_edgetpu.tflite"
)
DELEGATE    = "tpu"     # or "cpu"
CONF_THRESH = 0.05      # very low so we can see raw scores
VAL_IMAGE   = pathlib.Path(
    "/home/frederick/FusionZero-Robocup-International/5_ai_training_data/0_images/images/20250518_012837_212169.jpg"
)
WARMUP_RUNS = 5
# ───────────────────────────────────────────────────────────────────────────────

def load_interpreter():
    delegates = [load_delegate("libedgetpu.so.1")] if DELEGATE=="tpu" else []
    interp = Interpreter(str(MODEL_PATH), experimental_delegates=delegates)
    interp.allocate_tensors()
    return interp

def letterbox(img, size=(640,640)):
    h0, w0 = img.shape[:2]
    H, W   = size
    r = min(H/h0, W/w0)
    nh, nw = int(h0*r), int(w0*r)
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    pad = np.zeros((H, W, 3), dtype=img.dtype)
    dy, dx = (H-nh)//2, (W-nw)//2
    pad[dy:dy+nh, dx:dx+nw] = resized
    return pad, r, dx, dy

def debug_model(interp):
    inp = interp.get_input_details()[0]
    out = interp.get_output_details()[0]
    print("INPUT DETAILS :", inp)
    print("OUTPUT DETAILS:", out)

    # warm-up + timing
    shape, dtype, idx = inp["shape"], inp["dtype"], inp["index"]
    lo, hi = (0,256) if dtype==np.uint8 else (-128,128)
    rnd = np.random.randint(lo, hi, shape, dtype)
    for _ in range(WARMUP_RUNS):
        interp.set_tensor(idx, rnd); interp.invoke()
    t0 = time.perf_counter()
    interp.set_tensor(idx, rnd); interp.invoke()
    dt = (time.perf_counter() - t0)*1000
    out_data = interp.get_tensor(out["index"])
    print(f"RANDOM-INFER → {dt:.2f} ms, output shape {out_data.shape}")
    print("Sample raw output:", out_data.flatten()[:5])

def test_static_image(interp):
    inp = interp.get_input_details()[0]
    out = interp.get_output_details()[0]

    img = cv2.imread(str(VAL_IMAGE))
    # apply SAME crop & flips as capture:
    roi = img[:int(img.shape[1]/2) - 160, :]  # width=960→height slice=320
    roi = cv2.flip(roi, 0)
    roi = cv2.flip(roi, 1)
    cv2.imshow("ROI", roi); cv2.waitKey(500); cv2.destroyWindow("ROI")

    # letter-box + display
    H, W = inp["shape"][1:3]
    padded, r, dx, dy = letterbox(roi, (H, W))
    print("Padded image shape:", padded.shape,
          "range:", padded.min(), padded.max())
    cv2.imshow("Padded", padded); cv2.waitKey(500); cv2.destroyWindow("Padded")

    # use PyCoral to quantize & set input
    common.set_input(interp, padded)
    interp.invoke()

    dets = np.squeeze(interp.get_tensor(out["index"])).T  # [N,5]
    print("All raw scores:", dets[:,4][:10])
    print("All raw boxes:", dets[:5,:4])
    print("Filtered > %.2f:" % CONF_THRESH, dets[dets[:,4] > CONF_THRESH])

def main():
    interp = load_interpreter()
    print("\n── MODEL DEBUG ──")
    debug_model(interp)
    print("\n── STATIC IMAGE TEST ──")
    test_static_image(interp)
    print("\nIf you see non-empty scores above, your model is working!\n")

if __name__ == "__main__":
    main()
