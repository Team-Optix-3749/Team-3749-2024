import math

import numpy as np
import statsmodels.api as sm
import pandas as pd
from sklearn.metrics import mean_squared_error
import matplotlib.pyplot as plt

# Read data from CSV file
data = pd.read_csv('data3.csv')
x1 = data["4 Bar Position Deg"].values /180.0 * math.pi
x2 = data["Arm Position Deg"].values/180.0 * math.pi
y = data["Gravity Voltage"].values

# Create design matrix
X = np.column_stack((np.ones(x1.size),
                     x1,
                     x1**2,
                     x1**3,
                     x2,
                     x2**2,
                     x1*x2,
                     (x1**2)*(x2),
                     (x1**3)*(x2),
                     (x1)*(x2**2),
                     (x1**2)*(x2**2),
                     (x1**3)*(x2**2)))

# Fit the quadratic regression model
model = sm.OLS(y, X)
results = model.fit()
coefficients = results.params
predicted_y = results.predict(X);

# Define the variables
x1_str = 'x'
x2_str = 'y'

# Construct the equation string
print(coefficients)

equation = f'y_hat = {coefficients[0]:.4f} + '
equation += f'{coefficients[1]} * {x1_str} + '
equation += f'{coefficients[2]} * {x1_str}^2 + '
equation += f'{coefficients[3]} * {x1_str}^3 + '
equation += f'{coefficients[4]} * {x2_str} + '
equation += f'{coefficients[5]} * {x2_str}^2 + '
equation += f'{coefficients[6]} * {x1_str+x2_str} + '
equation += f'{coefficients[7]} * {x1_str+"^2"+x2_str} + '
equation += f'{coefficients[8]} * {x1_str+"^3"+x2_str} + '
equation += f'{coefficients[9]} * {x1_str+x2_str+"^2"} + '
equation += f'{coefficients[10]} * {x1_str+"^2"+x2_str+"^2"} '
equation += f'{coefficients[11]} * {x1_str+"^3"+x2_str+"^2"} '

rmse = mean_squared_error(y, predicted_y, squared=False)

# Print or use the equation string
print("Equation of the fitted quadratic regression model:")
print(equation)

print("R Squared")
print(results.rsquared)
print("R Squared Adjusted")
print(results.rsquared_adj)
print("Root Mean Squared Error")
print(rmse)

