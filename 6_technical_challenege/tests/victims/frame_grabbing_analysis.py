import sys, pathlib
root = pathlib.Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(root))

from hardware.robot import *
from core.utilities import *

from behaviours.optimized_evacuation import op_search

start_display()

import cv2
import os
import random
import sys

frames_dir = "/home/aidan/FusionZero-Robocup-International/1_international/tests/victims/frames"

frame_files = [f for f in os.listdir(frames_dir) if f.lower().endswith('.png')]
if not frame_files:
    print(f"No PNG files in {frames_dir}")
    sys.exit(1)

image_counter = 0

while True:
    print(f"Image number: {image_counter}")
    
    random_frame = random.choice(frame_files)
    image_path = os.path.join(frames_dir, random_frame)
    
    # Collect random image
    image = cv2.imread(image_path)
    display_image = image.copy()
    
    
    live_x = op_search.live(image, display_image, None)
    
    show(image)
    show(display_image)
    
    input(live_x)
    image_counter += 1