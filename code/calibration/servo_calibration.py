import numpy as np

calibration_values = open("calibration_values.txt", "r")

allTimes = calibration_values.read().split("\n")

frequencies = np.array([-50, -40, -30, -20, -10, 10, 20, 30, 40, 50])

for time in allTimes:
    time = time.split(",")
    time.pop()
    time = list(map(float, time))

    time = 
    
    print(time)