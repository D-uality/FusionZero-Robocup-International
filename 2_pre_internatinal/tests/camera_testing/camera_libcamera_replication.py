#!/usr/bin/env python3

import time
import cv2
from picamera2 import Picamera2

cv2.startWindowThread()

def main():
    # Create the camera instance.
    picam2 = Picamera2()

    # Create a still configuration that:
    #   1) Requests a main (final) stream: 2304×1296 @ YUV420
    #   2) Requests a RAW stream: 2304×1296 @ SBGGR10
    # The RAW request ensures the sensor is driven in the desired 2304×1296 SBGGR10_1X10 mode
    # while the main stream is output in 2304×1296 YUV420 (as seen in the libcamera-hello logs).
    config = picam2.create_still_configuration(
        main={"size": (320, 180), "format": "YUV420"},
        raw={"size": (2304, 1296), "format": "SBGGR10"}
    )


    # Apply the configuration and start the camera.
    picam2.configure(config)
    picam2.start()
    time.sleep(2)  # Let the sensor/AGC settle briefly

    print("Capturing frames at 2304×1296 (YUV420). Press 'q' to quit.")
    try:
        while True:
            start_time = time.time()
            
            # Grab a frame (the YUV420 stream) as a NumPy array.
            frame = picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_I420)
            cv2.imshow("Camera Stream", frame)

            # Press 'q' in the window to exit.
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            print(f"{1/(time.time() - start_time):.2f} fps")

    except KeyboardInterrupt:
        pass

    finally:
        picam2.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()