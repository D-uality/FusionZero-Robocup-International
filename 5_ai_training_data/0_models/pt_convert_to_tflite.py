from ultralytics import YOLO

# 1️⃣ Load the classifier (this auto-creates the architecture)
yolo = YOLO('/home/frederick/FusionZero-Robocup-International/5_ai_training_data/0_models/silver_classify_s.pt')

# 2️⃣ One-liner export straight to TFLite
#    imgsz is the square side length; 224 is typical for ImageNet-style classifiers.
yolo.export(format='tflite', imgsz=224, dynamic=True)   # creates silver_classify_s.tflite
