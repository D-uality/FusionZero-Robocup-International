import cv2
import numpy as np
from picamera2 import Picamera2
from libcamera import Transform

# Set up the camera
camera = Picamera2()
camera_config = camera.create_still_configuration(
    main={"size": (320, 200), "format": "YUV420"},
    raw={"size": (2304, 1500), "format": "SBGGR10"},
    transform=Transform(vflip=False, hflip=False)
)
camera.configure(camera_config)
camera.start()

# Capture the initial image and convert from YUV to BGR


# Start the window thread and create a display window
cv2.startWindowThread()
cv2.namedWindow("image")

# Dummy callback for the trackbars (not used)
def nothing(x):
    pass

# Create trackbars for Hough Circle parameters.
# For 'dp', we use a slider value divided by 10 to represent float values.
cv2.createTrackbar("dp", "image", 10, 50, nothing)         # dp range: 1.0 to 5.0 (10 -> 1.0, 50 -> 5.0)
cv2.createTrackbar("minDist", "image", 20, 300, nothing)     # Minimum distance between detected centers
cv2.createTrackbar("param1", "image", 50, 200, nothing)      # Higher threshold for Canny edge detector
cv2.createTrackbar("param2", "image", 30, 100, nothing)      # Accumulator threshold for circle detection
cv2.createTrackbar("minRadius", "image", 0, 100, nothing)    # Minimum circle radius
cv2.createTrackbar("maxRadius", "image", 0, 300, nothing)    # Maximum circle radius (0 means no limit)

def process_hough_transform(img, params):
    """Convert image to grayscale, blur it, perform Hough Circle Transform, and draw circles."""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)
    circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT,
                               dp=params["dp"],
                               minDist=params["minDist"],
                               param1=params["param1"],
                               param2=params["param2"],
                               minRadius=params["minRadius"],
                               maxRadius=params["maxRadius"])
    output = img.copy()
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for circle in circles[0, :]:
            center = (circle[0], circle[1])
            radius = circle[2]
            # Draw the outer circle (green) and the center (red)
            cv2.circle(output, center, radius, (0, 255, 0), 2)
            cv2.circle(output, center, 2, (0, 0, 255), 3)
    return output

while True:
    image = camera.capture_array()
    image = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_I420)
    image = image[20:, :]  # Crop if needed
    
    # Read parameter values from the trackbars
    params = {
        "dp": cv2.getTrackbarPos("dp", "image") / 10.0,
        "minDist": cv2.getTrackbarPos("minDist", "image"),
        "param1": cv2.getTrackbarPos("param1", "image"),
        "param2": cv2.getTrackbarPos("param2", "image"),
        "minRadius": cv2.getTrackbarPos("minRadius", "image"),
        "maxRadius": cv2.getTrackbarPos("maxRadius", "image")
    }
    
    # Process the image using the updated parameters
    processed_image = process_hough_transform(image, params)
    
    # Optionally overlay current parameter values on the image
    y0, dy = 20, 25
    for i, (key, value) in enumerate(params.items()):
        text = f"{key}: {value}"
        y = y0 + i * dy
        cv2.putText(processed_image, text, (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
    
    # Display the processed image
    cv2.imshow("image", processed_image)
    
    # Exit if the ESC key is pressed
    key = cv2.waitKey(100) & 0xFF
    if key == 27:
        break

cv2.destroyAllWindows()
