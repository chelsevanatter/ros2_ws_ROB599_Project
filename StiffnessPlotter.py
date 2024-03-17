import csv
import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import linregress

# Define the file name for the CSV file
CSV_FILE = 'arduino_data_subscriber.csv'

# Lists to store the data
all_load_cell_readings = []
all_motor_steps = []
load_cell_readings = []
motor_steps = []

# Read data from the CSV file
with open(CSV_FILE, 'r') as file:
    reader = csv.reader(file)
    next(reader)  # Skip the header row
    for row in reader:
        all_load_cell_readings.append(float(row[2]))  # Convert to float and append load cell reading
        all_motor_steps.append(float(row[3]))  # Convert to float and append motor steps
        if float(row[2]) > 50:
            load_cell_readings.append(float(row[2]))
            motor_steps.append(float(row[3]))

# Plot the data
plt.plot(motor_steps, load_cell_readings, marker='o', linestyle='-')
plt.title('Force vs. Displacement')
plt.xlabel('Displacement (mm)')
plt.ylabel('Force (g)')
plt.xlim([0, 50])
plt.ylim([0, 1200])
plt.grid(True)

# Perform linear regression
slope, intercept, r_value, p_value, std_err = linregress(motor_steps, load_cell_readings)
line = slope * np.array(motor_steps) + intercept
plt.plot(motor_steps, line, color='red', linestyle='--', label='Regression Line')

# Annotate the plot with slope and R-squared value
plt.text(20, 1000, f'Slope: {slope:.2f}', fontsize=12, color='blue')
plt.text(20, 800, f'R-squared: {r_value**2:.2f}', fontsize=12, color='blue')

plt.legend()
plt.show()

plt.plot(all_motor_steps, all_load_cell_readings, marker='o', linestyle='-')
plt.title('All Force vs. Displacement')
plt.xlabel('Displacement (mm)')
plt.ylabel('Force (g)')
plt.xlim([0, 50])
plt.ylim([0, 1200])
plt.grid(True)
plt.show()
