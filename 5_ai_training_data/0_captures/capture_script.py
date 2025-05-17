import cv2, sys, time, datetime, os

DEVICE_PATH = "/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0"
SAVE_DIR = "/home/frederick/FusionZero-Robocup-International/5_ai_training_data/captures"

def main() -> None:
    camera = cv2.VideoCapture(DEVICE_PATH, cv2.CAP_V4L2)
    if not camera.isOpened(): sys.exit("Cannot Open Camera!")

    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    camera.set(cv2.CAP_PROP_FPS, 30)
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    os.makedirs("/home/frederick/FusionZero-Robocup-International/5_ai_training_data/captures", exist_ok=True)

    try:
        while True:
            t0 = time.perf_counter()
            ok, image = camera.read()
            
            if not ok: continue
            
            image = image[:int(240/2) + 15][:]
            image = cv2.flip(image, 0)
            image = cv2.flip(image, 1)
            
            # Display image
            cv2.imshow("image", image)
            
            # exit on q
            user_input = cv2.waitKey(1) & 0xFF
            if  user_input == ord("q"): break
            
            elif user_input == ord("c"):
                print("Capture image")
                
                time_stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                path = os.path.join(SAVE_DIR, f"{time_stamp}.jpg")

                cv2.imwrite(path, image)
                print(f"Captured {path}")
            
            # print capture rate
            print(f"{1 / (time.perf_counter() - t0):.2f}")
            
    finally:
        camera.release()
        cv2.destroyAllWindows()

if __name__ == "__main__": main()