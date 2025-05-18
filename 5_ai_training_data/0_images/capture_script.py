import cv2, sys, time, datetime, os
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

BUTTON_PIN = 22
DEVICE_PATH = "/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0"
SAVE_DIR = "/home/frederick/FusionZero-Robocup-International/5_ai_training_data/0_images/images"

GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
prev_pressed = (GPIO.input(BUTTON_PIN) == GPIO.LOW)

def main() -> None:
    global prev_pressed
    
    camera = cv2.VideoCapture(DEVICE_PATH, cv2.CAP_V4L2)
    if not camera.isOpened(): sys.exit("Cannot Open Camera!")

    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    camera.set(cv2.CAP_PROP_FPS, 30)
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    os.makedirs(SAVE_DIR, exist_ok=True)
    file_count = len([f for f in os.listdir(SAVE_DIR) if os.path.isfile(os.path.join(SAVE_DIR, f))])
    print(f"Existing images: {file_count}")

    try:
        while True:
            t0 = time.perf_counter()
            ok, image = camera.read()
            if not ok: continue

            image = image[:int(960/2) - 160][:]
            image = cv2.flip(image, 0)
            image = cv2.flip(image, 1)

            cv2.imshow("image", image)
            user_input = cv2.waitKey(1) & 0xFF
            pressed = (GPIO.input(BUTTON_PIN) == GPIO.LOW)
            
            if user_input == ord("q"): break
            
            elif user_input == ord("c") or pressed != prev_pressed:
                prev_pressed = pressed
                
                time_stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                path = os.path.join(SAVE_DIR, f"{time_stamp}.jpg")
                
                cv2.imwrite(path, image)
                
                file_count += 1
                print(f"Captured {path}  |  total images: {file_count}")

            print(f"{1 / (time.perf_counter() - t0):.2f}")
    finally:
        camera.release()
        cv2.destroyAllWindows()

if __name__ == "__main__": main()
