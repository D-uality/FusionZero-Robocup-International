import sys, pathlib
root = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(root))

from core.utilities import *
from hardware.robot import *

# Config variables
TRANSFORM = False
WIDTH, HEIGHT = 320, 200  # Correct resolution for Pi Camera
FLIP = False
CROP_MIN, CROP_MAX = 0, HEIGHT

h_min, h_max = 0, 179
s_min, s_max = 0, 255
v_min, v_max = 0, 255

selectors = ["H_min", "H_max", "S_min", "S_max", "V_min", "V_max"]
current_selector = 0

def hsv_mask(frame):
    # Convert the frame to HSV
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define HSV range and create mask
    lower_bound = np.array([h_min, s_min, v_min])
    upper_bound = np.array([h_max, s_max, v_max])
    mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)

    # Apply mask to the frame
    result = cv2.bitwise_and(frame, frame, mask=mask)
    return mask, result

try:
    while True:
        # Capture image
        image = evac_camera.capture()

        # Apply HSV mask
        masked_image, mask_result = hsv_mask(image)

        # Display the frames
        cv2.imshow("Original Frame", image)
        cv2.imshow("HSV Mask", masked_image)
        cv2.imshow("Result", mask_result)

        # Display current thresholds and selector
        print(f"Adjusting: {selectors[current_selector]} | H_min={h_min}, H_max={h_max}, S_min={s_min}, S_max={s_max}, V_min={v_min}, V_max={v_max}")

        # Handle key presses
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):  # Quit
            break
        elif key == 82:  # Up arrow
            current_selector = (current_selector - 1) % len(selectors)
        elif key == 84:  # Down arrow
            current_selector = (current_selector + 1) % len(selectors)
        elif key == 81:  # Left arrow
            if selectors[current_selector] == "H_min":
                h_min = max(0, h_min - 1)
            elif selectors[current_selector] == "H_max":
                h_max = max(h_min, h_max - 1)
            elif selectors[current_selector] == "S_min":
                s_min = max(0, s_min - 1)
            elif selectors[current_selector] == "S_max":
                s_max = max(s_min, s_max - 1)
            elif selectors[current_selector] == "V_min":
                v_min = max(0, v_min - 1)
            elif selectors[current_selector] == "V_max":
                v_max = max(v_min, v_max - 1)
        elif key == 83:  # Right arrow
            if selectors[current_selector] == "H_min":
                h_min = min(h_max, h_min + 1)
            elif selectors[current_selector] == "H_max":
                h_max = min(179, h_max + 1)
            elif selectors[current_selector] == "S_min":
                s_min = min(s_max, s_min + 1)
            elif selectors[current_selector] == "S_max":
                s_max = min(255, s_max + 1)
            elif selectors[current_selector] == "V_min":
                v_min = min(v_max, v_min + 1)
            elif selectors[current_selector] == "V_max":
                v_max = min(255, v_max + 1)

except KeyboardInterrupt:
    print("Exiting loop...")

# Release the camera and close all OpenCV windows
cv2.destroyAllWindows()
evac_camera.__release()
