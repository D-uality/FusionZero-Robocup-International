calibration_values = open("calibration_values.txt", "r")

values = calibration_values.read()

servos = values.split("\n")

for servo in servos:
    servo = servo.split(',').

    print(servo)