# silver_detection.py
import threading, queue
from ultralytics import YOLO

image_queue       = queue.Queue(maxsize=1)
yolo_result_queue = queue.Queue(maxsize=1)

class YoloWorker(threading.Thread):
    def __init__(self, model_path: str, image_q=image_queue, result_q=yolo_result_queue):
        super().__init__(daemon=True)
        self.model        = YOLO(model_path)
        self.image_queue  = image_q      # ← store handles
        self.result_queue = result_q
        self.running      = True

    def run(self):
        while self.running:
            try:
                image = self.image_queue.get(timeout=0.5)
                results = self.model(image)[0]

                if results.probs is not None:
                    best_idx = int(results.probs.top1)
                    best_name = results.names[best_idx]
                    best_conf = float(results.probs.top1conf)
                else:
                    best_name = "None"
                    best_conf = 0.0

                if not self.result_queue.empty():
                    self.result_queue.get_nowait()
                self.result_queue.put((best_name, best_conf))

            except queue.Empty:
                continue


    def stop(self):
        self.running = False