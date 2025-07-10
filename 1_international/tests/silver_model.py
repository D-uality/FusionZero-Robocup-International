import sys, pathlib
root = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(root))

from core.utilities import *
from hardware.robot import *

from ultralytics import YOLO


start_display()

yolo = YOLO('/home/aidan/FusionZero-Robocup-International/5_ai_training_data/0_models/silver_classification.pt')
led.on()

print("program start")

while True:
    image = camera.capture_array()
    image = camera.perspective_transform(image)
    show(image, name="Display", display=True)
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