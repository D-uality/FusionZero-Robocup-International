import RPi.GPIO as GPIO
import time
from adafruit_servokit import ServoKit
from picamera2 import Picamera2
from libcamera import Transform
import cv2

# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(13, GPIO.OUT)

led_state = False  # Track LED state

def toggle_led():
    global led_state
    led_state = not led_state
    GPIO.output(13, GPIO.LOW if led_state else GPIO.HIGH)

def led_close():
    GPIO.cleanup()

# Servo Setup
pca = ServoKit(channels=16)
stop_speed = 95

def servo_write(left_speed, right_speed):
    left_speed = max(0, min(180, stop_speed + left_speed))
    right_speed = max(0, min(180, stop_speed - right_speed))
    
    pca.servo[14].angle = left_speed
    pca.servo[15].angle = left_speed
    pca.servo[12].angle = right_speed
    pca.servo[13].angle = right_speed

# Camera Setup
WIDTH, HEIGHT = 160, 120
FLIP = False
X11 = True

camera = Picamera2()
camera_config = camera.create_preview_configuration(
    main={"format": "RGB888", "size": (WIDTH, HEIGHT)},
    transform=Transform(vflip=FLIP, hflip=FLIP)
)
camera.configure(camera_config)
camera.start()

# Control Variables
speed = [90, 48]  # Adjust speeds
move_delay = 0.2  # Time for turning
current_direction = None  # Track current movement: 'forward', 'backward', or None

try:
    print("Use WASD to control the robot, SPACE to stop, 'l' to toggle LED, and 'q' to quit.")
    
    while True:
        # Camera preview
        image = camera.capture_array()
        cv2.imshow("Image", image)
        
        # Get key input using cv2.waitKey
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):  # Quit
            print("Exiting control.")
            break

        elif key == ord('w'):  # Forward
            servo_write(speed[0], speed[1])
            current_direction = 'forward'

        elif key == ord('s'):  # Backward
            servo_write(-speed[0], -speed[1])
            current_direction = 'backward'

        elif key == ord(' '):  # Stop
            servo_write(0, 0)
            current_direction = None

        elif key == ord('a'):  # Turn Left
            servo_write(int(-speed[0]/2), int(speed[1]/2))
            time.sleep(move_delay)
            if current_direction == 'forward':
                servo_write(speed[0], speed[1])
            elif current_direction == 'backward':
                servo_write(-speed[0], -speed[1])
            else:
                servo_write(0, 0)

        elif key == ord('d'):  # Turn Right
            servo_write(int(speed[0]/2), int(-speed[1]/2))
            time.sleep(move_delay)
            if current_direction == 'forward':
                servo_write(speed[0], speed[1])
            elif current_direction == 'backward':
                servo_write(-speed[0], -speed[1])
            else:
                servo_write(0, 0)

        elif key == ord('l'):  # Toggle LED
            toggle_led()

except KeyboardInterrupt:
    print("Exiting program...")

finally:
    # Cleanup
    led_close()
    servo_write(0, 0)
    camera.stop()
    cv2.destroyAllWindows()
