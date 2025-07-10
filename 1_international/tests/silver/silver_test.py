import sys, pathlib
root = pathlib.Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(root))

from hardware.robot import *
from core.utilities import *
from core.shared_imports import time

from silver_inference import SilverLineDetector
start_display()

detector = SilverLineDetector('/home/frederick/FusionZero-Robocup-International/5_ai_training_data/0_models/silver_line/silver_detector_pi4_quantized.pt')

led.on()
print("program start")

while True:
    image = camera.capture_array()
    image = camera.perspective_transform(image)
    
    show(image, name="Display", display=True)
    
    start_time = time.perf_counter()
    # Get prediction
    result = detector.predict(image)
    
    print(f"Prediction: {result['class_name']} (Confidence: {result['confidence']:.3f}) Time: {time.perf_counter() - start_time:.2f}ms")
    
    if result['prediction'] == 1 and result['confidence'] > 0.99:  # Silver detected
        print("Silver line detected!")
        
        
    