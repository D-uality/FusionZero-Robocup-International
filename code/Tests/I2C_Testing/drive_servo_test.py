import time
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)  # Update to 16 servos

# Midpoint value for stopping all servos
stopValue = 90
# FL = 97, FR = 84, BL = 95, BR = 83

# Function to set servo direction for left and right side servos
def set_servo_direction(num):
    # Ensure the input is within the valid range (0 to 180)
    if 0 <= num <= 180:
        # Set left side servos (14, 15) to the input number
        kit.servo[14].angle = num  # Left side forward/backward
        kit.servo[15].angle = num  # Left side forward/backward

        # Set right side servos (12, 13)
        if num == stopValue:
            # When num is stopValue (95), set right side servos to 95 as well
            kit.servo[12].angle = stopValue
            kit.servo[13].angle = stopValue
        else:
            # For all other inputs, use 197 - num for right side servos
            kit.servo[12].angle = max(0, min(180, 180 - num))  # Right side inverse
            kit.servo[13].angle = max(0, min(180, 180 - num))  # Right side inverse
    else:
        print("Invalid input! Angle must be between 0 and 180.")

# Main loop
while True:
    try:
        key = input("Enter num for servo direction (0-180): ")

        # Check if input is a valid number
        num = int(key)

        # Set the servo directions based on num
        set_servo_direction(num)

        time.sleep(1)
    except ValueError:
        print("Invalid input! Please enter a valid number between 0 and 180.")
