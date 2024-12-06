import time
from adafruit_servokit import ServoKit

servos = ServoKit(channels=16)

# servoPins = [14, 13, 12, 10]    # Frederick
servoPins = [15, 14, 13, 12]      # Aidan
calibration_values = open("calibration_values.txt", "w")

stop_angles = 97, 84, 95, 83

def controlLoop(servoPin, angle):
    servos.servo[servoPin].angle = angle

    print(f"\tPress enter to begin. Press enter again to stop.")
    input()

    startTime = time.time()

    input()

    endTime = time.time()

    elapsedTime = endTime - startTime
    servos.servo[servoPin].angle = stop_angle

    print(f"\t{elapsedTime}")
    return elapsedTime

try:
    mode = int(input("1 - All\n2 - Manual\nMode: "))

    if mode == 1:
        for servoPin in servoPins:
            for angle in range(stop_angle-50, stop_angle-9, 10):
                print(f"Servo: {servoPin} running at {angle}")

                elapsedTime = controlLoop(servoPin, angle)
                calibration_values.write(str(elapsedTime) + ", ")

                input()

            for angle in range(stop_angle+10, stop_angle+41, 10):
                print(f"Servo: {servoPin} running at {angle}")

                elapsedTime = controlLoop(servoPin, angle)
                calibration_values.write(str(elapsedTime) + ", ")

                input()

            calibration_values.write("\n")

    else:
        servoPin = servoPins[int(input("Servo Pin: "))]
        angle = int(input("Angle: "))

        elapsedTime = controlLoop(servoPin, angle)

except KeyboardInterrupt:
    print("Exiting Gracefully")

finally:
    for servoPin in servoPins:
        servos.servo[servoPin].angle = stop_angle

    calibration_values.close()