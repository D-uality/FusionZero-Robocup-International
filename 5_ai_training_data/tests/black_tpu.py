#!/usr/bin/env python3
import pathlib, sys, time
import cv2, numpy as np, tflite_runtime.interpreter as tflite

# ─── config ────────────────────────────────────────────────────────────────────
MODEL_PATH  = pathlib.Path("/home/frederick/FusionZero-Robocup-International/5_ai_training_data/models/dead_edgetpu.tflite")
DELEGATE    = "tpu"    # "cpu" or "tpu"
CONF_THRESH = 0.15     # lower for debug
DEVICE      = "/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0"
WARMUP_RUNS = 5
# ───────────────────────────────────────────────────────────────────────────────

def load_interpreter():
    if DELEGATE == "tpu":
        delegates = [tflite.load_delegate("libedgetpu.so.1")]
    else:
        delegates = []
    interp = tflite.Interpreter(str(MODEL_PATH),
                                experimental_delegates=delegates)
    interp.allocate_tensors()
    return interp

def letterbox(img, new_size=(640,640)):
    """
    Resize and pad img to new_size=(h,w), preserving aspect and centering.
    Returns padded, scale ratio, dx, dy for undoing later.
    """
    h0, w0 = img.shape[:2]
    h1, w1 = new_size
    r = min(h1/h0, w1/w0)
    nh, nw = int(h0*r), int(w0*r)
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    pad = np.zeros((h1, w1, 3), dtype=img.dtype)
    dy, dx = (h1-nh)//2, (w1-nw)//2
    pad[dy:dy+nh, dx:dx+nw] = resized
    return pad, r, dx, dy

def main():
    interp = load_interpreter()
    inp_det = interp.get_input_details()[0]
    out_det = interp.get_output_details()[0]
    dt      = inp_det["dtype"]
    h_in, w_in = inp_det["shape"][1:3]

    # warm-up
    lo, hi = (0,256) if dt==np.uint8 else (-128,128)
    sample = np.random.randint(lo, hi, inp_det["shape"], dt)
    for _ in range(WARMUP_RUNS):
        interp.set_tensor(inp_det["index"], sample)
        interp.invoke()

    cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
    if not cap.isOpened(): sys.exit("Cannot open camera!")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  960)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS,          30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)

    try:
        while True:
            t0, ret = time.perf_counter(), cap.read()[0]
            ret, frame = cap.read()
            if not ret: continue

            # 1) Crop & flip exactly as in your capture script
            roi = frame[:320, :]            # top 320 rows of 720
            roi = cv2.flip(roi, 0)          # vertical flip
            roi = cv2.flip(roi, 1)          # horizontal flip

            # 2) Letter-box to 640×640
            img, r, dx, dy = letterbox(roi, (640,640))
            inp = np.expand_dims(img, axis=0).astype(dt)
            interp.set_tensor(inp_det["index"], inp)

            # 3) Inference + unpack YOLOv8 single-tensor output
            interp.invoke()
            raw  = interp.get_tensor(out_det["index"])  # [1,5,N]
            dets = np.squeeze(raw).T                   # [N,5]: x,y,w,h,score

            # 4) Draw on the original ROI
            h0, w0 = roi.shape[:2]
            for x,y,w,h,score in dets:
                if score < CONF_THRESH: continue
                
                print("passing!")
                # coords in padded 640×640 image:
                x0 = (x - w/2)*w_in; y0 = (y - h/2)*h_in
                x1 = (x + w/2)*w_in; y1 = (y + h/2)*h_in
                # undo pad+scale → back to roi coords
                fx0 = int((x0 - dx)/r); fy0 = int((y0 - dy)/r)
                fx1 = int((x1 - dx)/r); fy1 = int((y1 - dy)/r)
                cv2.rectangle(roi, (fx0,fy0), (fx1,fy1), (0,255,0), 2)
                cv2.putText(roi, f"{score:.2f}", (fx0, fy0-5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

            # 5) Show FPS + window
            fps = 1.0 / (time.perf_counter() - t0)
            cv2.putText(roi, f"FPS:{fps:.1f}", (5,15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)
            cv2.imshow("TPU Inference", roi)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
