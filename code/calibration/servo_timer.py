import time
from adafruit_servokit import ServoKit

pca = ServoKit(channels=16)
servo_pins = [14, 13, 12, 10]
stop_angles = [97, 96, 96, 97]

def control_loop(pin_number, servo_pin, angle):
    pca.servo[servo_pin].angle = angle

    print(f"\tPress enter to begin. Press enter again to stop.")
    input()

    start_time = time.time()

    print(f"\t\tBegan recording time")
    input()

    elapsed_time = time.time() - start_time
    print(f"\t\tFinished recording time")

    pca.servo[servo_pin].angle = stop_angles[pin_number]

    return elapsed_time

try:
    while True:
        for servo_number, servo_pin in enumerate(servo_pins):
            pca.servo[servo_pin].angle = stop_angles[servo_number]

        print("1: Record for ALL servos\n2: Record for SPECIFIC servo\nMode: ")
        mode = int(input())

        if mode == 1:
            calibration_values = open("/home/fusion/FusionZero-Robocup-International/code/calibration/calibration_values.txt", "a")

            for pin_number, servo_pin in enumerate(servo_pins):
                stop_angle = stop_angles[pin_number]

                for angle in range(stop_angle-50, stop_angle+51, 10):
                    if angle == stop_angle: continue

                    print(f"Servo {servo_number} @ {angle}")

                    elapsed_time = control_loop(pin_number, servo_pin, angle)
                    calibration_values.write(f"{elapsed_time:.2f} ")
                    print(f"Saved: {elapsed_time:.2f}")

                    input("Waiting for next angle")

                calibration_values.write(f"\n")
                input("Waiting for next servo")

            calibration_values.close()

        if mode == 2:
            pin_number = int(input("Pin Number: "))
            angle = int(input("Step: ")) * 10 + stop_angles[pin_number]

            elapsed_time = control_loop(pin_number, servo_pins[pin_number], angle)
            print(f"{elapsed_time:.2f}")

except KeyboardInterrupt:
    print("Exiting Gracefully")

finally:
    for pin_number, servo_pin in enumerate(servo_pins):
        pca.servo[servo_pin].angle = stop_angles[pin_number]

        # messed up:
        # FR 106?