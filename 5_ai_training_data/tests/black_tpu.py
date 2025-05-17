import cv2, time, numpy as np, sys, tflite_runtime.interpreter as tflite
from collections import deque

modelPath  = "/home/frederick/FusionZero-Robocup-International/5_ai_training_data/models/black_edgetpu.tflite"
DEVICE_PATH = "/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0"
delegate   = "tpu"

def main():
    useTPU = delegate == "tpu"
    interpreter = tflite.Interpreter(
        model_path=modelPath,
        experimental_delegates=[tflite.load_delegate('libedgetpu.so.1')] if useTPU else None
    )
    interpreter.allocate_tensors()
    input_shape  = interpreter.get_input_details()[0]
    output_shape = interpreter.get_output_details()[0]
    h, w = map(int, input_shape["shape"][1:3])

    camera = cv2.VideoCapture(DEVICE_PATH, cv2.CAP_V4L2)
    if not camera.isOpened():
        sys.exit("Cannot Open Camera!")
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    camera.set(cv2.CAP_PROP_FPS, 30)
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    buf, i = deque(maxlen=50), 0

    while True:
        ok, frame = camera.read()
        if not ok or frame is None:
            continue

        frame = frame[:135, :]
        frame = cv2.flip(frame, 0)
        frame = cv2.flip(frame, 1)

        if frame.shape[0] != h or frame.shape[1] != w:
            frame = cv2.resize(frame, (w, h), interpolation=cv2.INTER_LINEAR)

        display = frame.copy()  # BGR for imshow
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB).astype(input_shape["dtype"])
        img = np.expand_dims(img, 0)

        CONF_THRESH = 0.55          # pick any 0‒1 value you like

        # --- inference ---
        interpreter.set_tensor(input_shape["index"], img)
        interpreter.invoke()
        raw_q = interpreter.get_tensor(output_shape["index"]).squeeze()     # (5,2100)

        # 1) de-quantise
        scale, zero = output_shape["quantization"]
        raw_f = (raw_q.astype(np.float32) - zero) * scale                   # float32 (5,2100)

        # 2) move to (2100,5) and split
        dets   = raw_f.T
        cxcywh = dets[:, :4]
        scores = dets[:, 4]

        # 3) sigmoid on confidence   (optionally cx,cy too)
        scores = 1 / (1 + np.exp(-scores))

        # keep anchors above threshold
        keep = scores > CONF_THRESH
        cxcywh, scores = cxcywh[keep], scores[keep]

        if cxcywh.size:
            # coords are 0-1 relative to model input
            cxcywh[:, [0, 2]] *= w
            cxcywh[:, [1, 3]] *= h
            x1y1 = cxcywh[:, :2] - cxcywh[:, 2:] / 2
            x2y2 = cxcywh[:, :2] + cxcywh[:, 2:] / 2

            for (x1, y1), (x2, y2), sc in zip(x1y1.astype(int),
                                            x2y2.astype(int), scores):
                cv2.rectangle(display, (x1, y1), (x2, y2), (0,255,0), 2)
                cv2.putText(display, f"{sc:.2f}", (x1, y1-8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0,255,0), 1, cv2.LINE_AA)


        # -------------------------------------

        cv2.imshow("TPU stream", display)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        buf.append(time.perf_counter())
        i += 1
        if i % 30 == 0 and len(buf) > 1:
            fps = (len(buf) - 1) / (buf[-1] - buf[0])
            print(f"FPS {fps:5.1f}")


    camera.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()