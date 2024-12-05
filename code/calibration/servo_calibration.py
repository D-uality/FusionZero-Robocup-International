import numpy as np
from sklearn.linear_model import LinearRegression

calibration_values = open("calibration_values.txt", "r")
allTimes = calibration_values.read().split("\n")

negativeGradients  = []
positiveGradients  = []
negativeIntercepts = []
positiveIntercepts = []

# For each servoTime
for time in allTimes:
    index = allTimes.index(time)
    # define the frequiencies used
    frequencies = np.array([-50, -40, -30, -20, -10, 10, 20, 30, 40, 50])

    # Read the calibration_values.txt
    times = time.strip().split(',')[:-1]
    times = list(map(float, times))

    times = np.array(times)

    omega = 10 * np.pi / times
    frequencies, omega = omega, frequencies

    negativeFrequencies = frequencies[times < 0]
    negativeOmega = omega[times < 0]
    posititiveFrequencies = frequencies[times > 0]
    positiveOmega = omega[times > 0]

    # Fit a linear regression model for both sets
    negativeModel = LinearRegression()
    negativeModel.fit(negativeFrequencies.reshape(-1, 1), negativeOmega)
    negativeOmegaPredictions = negativeModel.predict(negativeFrequencies.reshape(-1, 1))

    positiveModel = LinearRegression()
    positiveModel.fit(posititiveFrequencies.reshape(-1, 1), positiveOmega)
    positiveOmegaPredictions = positiveModel.predict(posititiveFrequencies.reshape(-1, 1))

    # Printing equations
    negativeSlope = negativeModel.coef_[0]
    negativeIntercept = negativeModel.intercept_
    positiveSlope = positiveModel.coef_[0]
    positiveIntercept = positiveModel.intercept_

    negativeGradients.append(negativeSlope)
    negativeIntercepts.append(negativeIntercept)
    positiveGradients.append(positiveSlope)
    positiveIntercepts.append(positiveIntercept)

    # Create the equation strings
    negative_equation = f"y = {negativeSlope:.2f}x + {negativeIntercept:.2f}"
    positive_equation = f"y = {positiveSlope:.2f}x + {positiveIntercept:.2f}"

    print(f"{index}: Negative: {negative_equation} Positive: {positive_equation}")

print([f"{x:.4f}" for x in negativeGradients])
print([f"{x:.4f}" for x in negativeIntercepts])
print([f"{x:.4f}" for x in positiveGradients])
print([f"{x:.4f}" for x in positiveIntercepts])