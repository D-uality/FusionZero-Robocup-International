import cv2, sys, time

DEVICE_PATH = "/dev/video0"

def main() -> None:
    camera = cv2.VideoCapture(DEVICE_PATH, cv2.CAP_V4L2)
    if not camera.isOpened(): sys.exit("Cannot Open Camera!")

    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    camera.set(cv2.CAP_PROP_FPS, 30)
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    try:
        while True:
            t0 = time.perf_counter()
            ok, image = camera.read()
            
            if not ok: continue
            
            # Display image
            image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
            image = cv2.resize(image, (320, 240), interpolation=cv2.INTER_AREA)
            cv2.imshow("image", image)
            
            # exit on q
            user_input = cv2.waitKey(1) & 0xFF
            if  user_input == ord("q"): break
            
            elif user_input == ord("c"):
                print("Capture image")
            
            # print capture rate
            print(f"{1 / (time.perf_counter() - t0):.2f}")
            
    finally:
        camera.release()
        cv2.destroyAllWindows()

if __name__ == "__main__": main()