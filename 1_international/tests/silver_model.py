import sys, pathlib
root = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(root))

from core.utilities import *
from hardware.robot import *

from ultralytics import YOLO


start_display()

yolo = YOLO('/home/frederick/FusionZero-Robocup-International/5_ai_training_data/0_models/silver_classify_s.pt')
led.off()

print("program start")

while True:
    image = evac_camera.capture()
    results = yolo(image)

    result = results[0]            # the Results object for this frame
    probs  = result.probs          # ultralytics.engine.results.Probs

    # index of the class with the highest probability
    best_idx = int(probs.top1)     

    # confidence for that class (float32)
    best_conf = float(probs.top1conf)

    # human-readable class name from the model’s label map
    best_name = result.names[best_idx]

    print(f"{best_name} {best_conf:.2%}")     # e.g. “Line 99.87%”