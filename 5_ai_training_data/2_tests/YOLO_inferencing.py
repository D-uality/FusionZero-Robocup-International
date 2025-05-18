#!/usr/bin/env python3
from ultralytics import YOLO
import cv2, glob, random, os

# ─── CONFIG ────────────────────────────────────────────────────────────────────
# Path to your trained TFLite model (plain INT-8)
MODEL_PATH    = "/home/frederick/FusionZero-Robocup-International/5_ai_training_data/0_models/dead_edgetpu.tflite"
# Folder containing your val images
VAL_IMAGES    = "/home/frederick/FusionZero-Robocup-International/5_ai_training_data/0_images/images"
# Camera device (can be index 0 or your /dev/v4l/... path)
CAMERA_SOURCE = "/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0"
# Inference settings
IMG_SIZE      = 640      # must match your train/export size
CONF_THR      = 0.3      # confidence threshold
# ───────────────────────────────────────────────────────────────────────────────

def main():
    # 1) verify model exists
    if not os.path.isfile(MODEL_PATH):
        print(f"Model not found: {MODEL_PATH}")
        return

    # 2) load model for CPU detections
    model = YOLO(MODEL_PATH, task='detect')

    # 3) run one-off random val-image inference
    img_files = glob.glob(os.path.join(VAL_IMAGES, "*.jpg"))
    if img_files:
        sample = random.choice(img_files)
        print("Running inference on:", sample)
        results = model(sample, imgsz=IMG_SIZE, conf=CONF_THR)
        annotated = results[0].plot()  # RGB H×W×3
        bgr = cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR)
        cv2.imshow("Sample Detection", bgr)
        cv2.waitKey(0)
        cv2.destroyWindow("Sample Detection")
    else:
        print(f"No validation images in {VAL_IMAGES}")

    # 4) now live camera loop
    cap = cv2.VideoCapture(CAMERA_SOURCE, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"Cannot open camera: {CAMERA_SOURCE}")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    print("Starting live camera inference. Press 'q' to quit.")
    while True:
        ret, frame = cap.read()
        if not ret: continue

        frame = frame[:320][:]
        frame = cv2.flip(frame, 0)
        frame = cv2.flip(frame, 1)

        # run detection
        results = model(frame, imgsz=IMG_SIZE, conf=CONF_THR)
        annotated = results[0].plot()

        cv2.imshow("Live Detections", annotated)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
