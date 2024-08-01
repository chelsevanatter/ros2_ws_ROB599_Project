#!/usr/bin/env python3

# StiffnessPlotter.py
#
# Chelse VanAtter
#
# Code to plot displacement vs force and calculate sitffness from csv files

# Import csv library, plotting library, numpy, and linear regression tools
import csv
from matplotlib import pyplot as plt 
from matplotlib.backends.backend_pdf import PdfPages

import numpy as np
from scipy.stats import linregress

stiffness_array = []
height_array = []
diameter_array = [4.94, 4.4, 5.3, 10.9, 10.4, 12.7, 13.7, 18.2, 20.8, 4.4, 6.7, 6.8, 9.7, 10, 15, 15.3, 15.9, 16, 6.2, 7.1, 9.1, 8.7, 9.4, 10.2, 13.5, 14.3, 14.1]
bush_4_diameters = [4.7, 4.9, 5, 9.8, 12.6, 12.3, 11.7, 12.6, 13.6]
diameter_array.append(bush_4_diameters)

# Set up the figure size and adjust the padding between and around the subplots
plt.rcParams["figure.figsize"] = [7.00, 3.50] 
plt.rcParams["figure.autolayout"] = True


for bush in range(4,5):
    for branch in range(1,4):
        for trial in range(1,4):
            # Define the file name for the CSV file
            CSV_FILE = f'/home/chelse/ros2_ws_ROB599_Project/csv_files/bush_{bush}_branch_{branch}_trial_{trial}.csv'

            # Declare lists to store the load cell force data and motor steps displacmenet data
            all_load_cell_readings = []
            all_motor_steps = []

            # Declare lists that will only have force readings greater than 50 gf since data below 50 gf is noisy
            load_cell_readings = []
            motor_steps = []

            # Read data from the CSV file
            with open(CSV_FILE, 'r') as file:
                reader = csv.reader(file)
                next(reader)  # Skip the header row
                for row in reader:
                    all_load_cell_readings.append(float(row[2]))  # Convert to float and append load cell reading
                    all_motor_steps.append(float(row[3]))  # Convert to float and append motor steps
                    if float(row[2]) > 20:
                        load_cell_readings.append(float(row[2]))
                        motor_steps.append(float(row[3]))
                height_array.append(float(row[4]))
                
            # Declare whether or not you want to make filtered and/or unfiltered plots
            make_filtered_plot = True
            make_unfiltered_plot = False

            if len(motor_steps)>0:
            
                # Calculate the offset for the motor steps
                offset = motor_steps[0] - 0.5
                # Subtract the offset from every value of the motor_steps array
                motor_steps = [step - offset for step in motor_steps]

                # Perform linear regression
                slope, intercept, r_value, p_value, std_err = linregress(motor_steps, load_cell_readings)
                line = slope * np.array(motor_steps) + intercept
                stiffness_array.append(slope)
            
                if make_filtered_plot: 
                    filtered_var_name = 'filtered_fig' + str(bush) + str(branch) + str(trial)
                    filtered_var_name = plt.figure()
                    # Plot the offset displacement and filtered force data for points that had a force reading greater than 50 gf
                    plt.plot(motor_steps, load_cell_readings, marker='o', linestyle='-')
                    plt.title(f'Filtered Force vs. Displacement for bush {bush}, branch {branch},trial {trial}')
                    plt.xlabel('Displacement (mm)')
                    plt.ylabel('Force (gf)')
                    plt.xlim([0, 50])
                    plt.ylim([0, 1200])
                    plt.grid(True)
                    plt.plot(motor_steps, line, color='red', linestyle='--', label='Regression Line')

                    # Annotate the plot with slope and R-squared value
                    plt.text(20, 1000, f'Slope/Stiffness: {slope:.2f}', fontsize=12, color='blue')
                    plt.text(20, 800, f'R-squared: {r_value**2:.2f}', fontsize=12, color='blue')

                    plt.legend()
                    #plt.show()

                # Perform linear regression
                slope, intercept, r_value, p_value, std_err = linregress(all_motor_steps, all_load_cell_readings)
                line = slope * np.array(all_motor_steps) + intercept
                

                
                if make_unfiltered_plot:
                    unfiltered_var_name = 'unfiltered_fig' + str(bush) + str(branch) + str(trial)
                    unfiltered_var_name = plt.figure()
                    # Plot the unfiltered data without linear regression
                    plt.plot(all_motor_steps, all_load_cell_readings, marker='o', linestyle='-')
                    plt.title(f'Unfiltered Force vs. Displacement for bush {bush}, branch {branch},trial {trial}')
                    plt.xlabel('Displacement (mm)')
                    plt.ylabel('Force (gf)')
                    plt.xlim([0, 300])
                    plt.ylim([0, 1200])
                    plt.grid(True)

                    # Annotate the plot with slope and R-squared value
                    plt.text(20, 1000, f'Slope/Stiffness: {slope:.2f}', fontsize=12, color='blue')
                    plt.text(20, 800, f'R-squared: {r_value**2:.2f}', fontsize=12, color='blue')
                    plt.plot(all_motor_steps, line, color='red', linestyle='--', label='Regression Line')
                    
                    plt.legend()
                    #plt.show()
            else:
                stiffness_array.append(0)
                if make_filtered_plot: 
                    filtered_var_name = 'filtered_fig' + str(bush) + str(branch) + str(trial)
                    filtered_var_name = plt.figure()
                    # Plot the offset displacement and filtered force data for points that had a force reading greater than 50 gf
                    plt.plot([0, 0], marker='o', linestyle='-')
                    plt.title(f'No data greater than 20 gf for bush {bush}, branch {branch},trial {trial}')
                if make_unfiltered_plot:
                    unfiltered_var_name = 'unfiltered_fig' + str(bush) + str(branch) + str(trial)
                    unfiltered_var_name = plt.figure()
                    # Plot the unfiltered data without linear regression
                    plt.plot(all_motor_steps, all_load_cell_readings, marker='o', linestyle='-')
                    plt.title(f'Unfiltered Force vs. Displacement for bush {bush}, branch {branch},trial {trial}')
                    plt.xlabel('Displacement (mm)')
                    plt.ylabel('Force (gf)')
                    plt.xlim([0, 300])
                    plt.ylim([0, 1200])
                    plt.grid(True)

                    # Annotate the plot with slope and R-squared value
                    plt.text(20, 1000, f'Slope/Stiffness: {slope:.2f}', fontsize=12, color='blue')
                    plt.text(20, 800, f'R-squared: {r_value**2:.2f}', fontsize=12, color='blue')
                    plt.plot(all_motor_steps, line, color='red', linestyle='--', label='Regression Line')
                    
                    plt.legend()
                    #plt.show()
            

print(len(stiffness_array))
print(len(height_array))
print(len(diameter_array))

import numpy as np

# Define the bin edges based on height ranges
bin_edges_height = list(range(0, 1100, 100))  # Bin edges from 0 to 1000 with a width of 100
bin_edges_stiffness = list(range(0, 160, 10))  # Bin edges for stiffness values
bin_edges_diameter = list(range(0, 20, 1))  # Bin edges for stiffness values

fig1000 = plt.figure()
# Scatterplot
plt.plot(height_array, stiffness_array, marker='o', linestyle='', color='blue')
plt.title('Stiffness (gf/mm) vs. Height (mm)')
plt.xlabel('Height (mm)')
plt.ylabel('Stiffness (gf/mm)')
plt.xlim([600, 1000])
plt.ylim([0, 150])
plt.grid(True)
#plt.show()

fig1001 = plt.figure()
# Histogram
plt.hist2d(height_array, stiffness_array, bins=[bin_edges_height, bin_edges_stiffness], cmap='Blues')
plt.colorbar(label='Frequency')
plt.title('Stiffness Histogram vs. Height Ranges')
plt.xlabel('Height (mm)')
plt.ylabel('Stiffness (gf/mm)')

plt.tight_layout()
#plt.show()

# name your Pdf file 
filename = "test_Data_analysis.pdf"  
p = PdfPages(filename) 
      
# get_fignums Return list of existing  
# figure numbers 
fig_nums = plt.get_fignums()   
figs = [plt.figure(n) for n in fig_nums] 
      
# iterating over the numbers in list 
for fig in figs:  
        
    # and saving the files 
    fig.savefig(p, format='pdf')  
      
# close the object 
p.close()   
  

  


'''
# Scatterplot
plt.plot(diameter_array, stiffness_array, marker='o', linestyle='', color='blue')
plt.title('Stiffness (gf/mm) vs. Height (mm)')
plt.xlabel('Diameter (mm)')
plt.ylabel('Stiffness (gf/mm)')
plt.xlim([0, 20])
plt.ylim([0, 150])
plt.grid(True)
plt.show()

# Histogram
plt.hist2d(diameter_array, stiffness_array, bins=[bin_edges_diameter, bin_edges_stiffness], cmap='Blues')
plt.colorbar(label='Frequency')
plt.title('Stiffness Histogram vs. Diameter Ranges')
plt.xlabel('Diameter (mm)')
plt.ylabel('Stiffness (gf/mm)')

plt.tight_layout()
plt.show()
'''