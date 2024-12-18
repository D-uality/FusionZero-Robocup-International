import RPi.GPIO as GPIO
import time
from adafruit_servokit import ServoKit
from picamera2 import Picamera2
from libcamera import Transform
import sys
import termios
import tty
import cv2

# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(13, GPIO.OUT)

led_state = False  # Track LED state

def toggle_led():
    global led_state
    led_state = not led_state
    GPIO.output(13, GPIO.Low if led_state else GPIO.HIGH)

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
WIDTH, HEIGHT = 640, 480
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

# Function to read a single character from the terminal
def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch
    
try:
    print("Use WASD to control the robot, SPACE to stop, 'l' to toggle LED, and 'q' to quit.")
    while True:
        # Camera preview
        if X11:
            image = camera.capture_array()
            cv2.imshow("Image", image)
            cv2.waitKey(1)

        # Get key input
        key = get_key()

        if key == 'w':  # Forward
            servo_write(speed[0], speed[1])
            current_direction = 'forward'

        elif key == 's':  # Backward
            servo_write(-speed[0], -speed[1])
            current_direction = 'backward'

        elif key == ' ':  # Stop
            servo_write(0, 0)
            current_direction = None

        elif key == 'a':  # Turn Left
            servo_write(int(-speed[0]/2),int(speed[1]/2))
            time.sleep(move_delay)
            if current_direction == 'forward':
                servo_write(speed[0], speed[1])
            elif current_direction == 'backward':
                servo_write(-speed[0], -speed[1])
            else:
                servo_write(0, 0)

        elif key == 'd':  # Turn Right
            servo_write(int(speed[0]/2), int(-speed[1]/2))
            time.sleep(move_delay)
            if current_direction == 'forward':
                servo_write(speed[0], speed[1])
            elif current_direction == 'backward':
                servo_write(-speed[0], -speed[1])
            else:
                servo_write(0, 0)

        elif key == 'l':  # Toggle LED
            toggle_led()

        elif key == 'q':  # Quit
            print("Exiting control.")
            break

except KeyboardInterrupt:
    print("Exiting program...")

finally:
    # Cleanup
    led_close()
    servo_write(0, 0)
    camera.stop()
    cv2.destroyAllWindows()
