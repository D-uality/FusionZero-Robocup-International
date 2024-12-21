import time
import numpy as np
import cv2
from picamera2 import Picamera2
from libcamera import Transform
from ultralytics import YOLO

# Config variables
WIDTH, HEIGHT = 120, 90  # Correct resolution for Pi Camera
FLIP = False

# Load your YOLO model (Edge TPU version)
model = YOLO("/home/fusion/FusionZero-Robocup-International/code/tests/camera/inference/best_saved_model/live_victim_full_integer_quant_edgetpu.tflite")

# Initialize Picamera2
camera = Picamera2()

# Create a correct transform object from libcamera.Transform
camera_config = camera.create_preview_configuration(main={"format": "RGB888", "size": (WIDTH, HEIGHT)}, transform=Transform(vflip=FLIP, hflip=FLIP))

# Configure and start the camera
camera.configure(camera_config)
camera.start()

while True:
    time_start = time.time()  # Start timing FPS calculation

    # Capture image
    image = camera.capture_array()

    # Run the YOLO model on the captured image
    results = model.predict(image, workers=4)  # Use image directly, no need to save it

    # Process the results and draw bounding boxes
    detected_image = image.copy()  # Create a copy to draw on

    for result in results[0].boxes.data:  # Access the detection results
        x1, y1, x2, y2, conf, cls = result  # Unpack the result
        print(f"Detected object: Class {int(cls)}, Confidence {conf:.2f}")
        print(f"Bounding Box: x1={x1}, y1={y1}, x2={x2}, y2={y2}")
        
        # Draw bounding box and label
        cv2.rectangle(detected_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        cv2.putText(detected_image, f'{conf:.2f}', (int(x1), int(y1) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Display the original and detected images side by side
    combined_image = np.hstack((image, detected_image))  # Combine original and detected image horizontally
    cv2.imshow('Original and Detected Images', combined_image)

    # Wait for key press to close window
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    fps = 1 / (time.time() - time_start)

    print(f"FPS: {fps:.2f}")

# Clean up
cv2.destroyAllWindows()
