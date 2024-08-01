#!/usr/bin/env python3

# StiffnessPlotter.py
#
# Chelse VanAtter
#
# Code to plot displacement vs force and calculate sitffness from csv files
# Note about units, force was measured in grams force but is converted to millinewtons here, displacement, height, and diameter are in millimeters

# Import csv library, plotting library, numpy, and linear regression tools
import csv
from matplotlib import pyplot as plt 
from matplotlib.backends.backend_pdf import PdfPages

import numpy as np
from scipy.stats import linregress

# important values to change

# Note it is less than or equal to this value that will be flagged as bad
r_value_threshold = 0.97

# Declare whether or not you want to make filtered and/or unfiltered plots
make_filtered_plot = True
make_unfiltered_plot = True

# name your Pdf file 
filename = "test_New_data_analysis.pdf"  

# Set the marker type
marker_type = 'o'

# Declare whether or not you want the final plots to have slope for all data and good data or just all data
show_good_slope = False

# Graph axes:
displacement_axis = [0, 100]
filtered_force_axis = [0,12000]
unfiltered_force_axis = [-2000, 12000]
height_axis = [600,1100]
diameter_axis = [0,20]
stiffness_axis = [0,1400]

# Define the bin edges based on height ranges
bin_edges_stiffness = list(range(0, 1400, 50))  # Bin edges for stiffness values


stiffness_array = []
height_array = []
diameter_array = []
delta_x_array =[]
delta_F_array = []
max_F_array = []
bush_1_diameters = [4.94, 4.4, 5.3, 10.9, 10.4, 12.7, 13.7, 18.2, 20.8]
bush_2_diameters = [4.4, 6.7, 6.8, 9.7, 10, 15, 15.3, 15.9, 16]
bush_3_diameters = [6.2, 7.1, 9.1, 8.7, 9.4, 10.2, 13.5, 14.3, 14.1]
bush_4_diameters = [4.7, 4.9, 5, 9.8, 12.6, 12.3, 11.7, 12.6, 13.6]
bush_5_diameters = [7.7, 7.3, 7.9, 5.8, 9.3, 10.5, 16.1, 18, 22.6]
bush_6_diameters = [8.5, 12.8, 12.4, 11.9, 12.3, 12.3, 11.7, 11.8, 12.3]
diameter_array = [*bush_1_diameters, *bush_2_diameters, *bush_3_diameters, *bush_4_diameters, *bush_5_diameters, *bush_6_diameters]

bad_stiffness_array = []
bad_height_array = []
bad_diameter_array = []
good_stiffness_array = []
good_height_array = []
good_diameter_array = []
bad_index_array = []

# Set up the figure size and adjust the padding between and around the subplots
plt.rcParams["figure.figsize"] = [7.00, 3.50] 
plt.rcParams["figure.autolayout"] = True


for bush in range(1,7):
    for branch in range(1,4):
        for trial in range(1,4):
            # Define the file name for the CSV file
            CSV_FILE = f'/home/chelse/ros2_ws_ROB599_Project/csv_files/bush_{bush}_branch_{branch}_trial_{trial}.csv'

            # Declare lists to store the load cell force data and motor steps displacmenet data
            all_load_cell_readings = []
            all_motor_steps = []

            # Declare lists that will only have force readings greater than 20 gf since data below 20 gf is noisy
            load_cell_readings = []
            motor_steps = []

            # Read data from the CSV file
            with open(CSV_FILE, 'r') as file:
                reader = csv.reader(file)
                next(reader)  # Skip the header row
                for row in reader:
                    all_load_cell_readings.append((float(row[2]))*9.80665)  # Convert to float, multiply by conversion factor to convert from gf to mN, and append load cell reading 
                    all_motor_steps.append(float(row[3]))  # Convert to float and append motor steps
                    if float(row[2]) > 20:
                        load_cell_readings.append((float(row[2]))*9.80665)
                        motor_steps.append(float(row[3]))
                height_array.append(float(row[4]))

            # Calculate the offset for the all of the motor steps
            offset = all_motor_steps[0] - 0.5
            # Subtract the offset from every value of the all_motor_steps array
            all_motor_steps = [step - offset for step in all_motor_steps]

            if len(motor_steps) > 0:
            
                # Calculate the offset for the motor steps
                offset = motor_steps[0] - 0.5
                # Subtract the offset from every value of the motor_steps array
                motor_steps = [step - offset for step in motor_steps]

                # Get the delta x
                delta_x = round(max(motor_steps) - min(motor_steps),2)
                delta_x_array.append(delta_x)

                # Get the delta F
                delta_F = round(max(load_cell_readings) - min(load_cell_readings),2)
                delta_F_array.append(delta_F)

                # Get the max F
                max_F = round(max(load_cell_readings),2)
                max_F_array.append(max_F)

                # Perform linear regression
                slope, intercept, r_value, p_value, std_err = linregress(motor_steps, load_cell_readings)
                line = slope * np.array(motor_steps) + intercept
                stiffness_array.append(slope)
                if r_value <= r_value_threshold:
                    bad_stiffness_array.append(slope)
                    bad_height_array.append(height_array[-1])
                    diameter_index = ((bush * 9) - 9) + ((branch * 3) - 3) + (trial - 1)
                    bad_diameter_array.append(diameter_array[diameter_index])
                    bad_index_array.append(diameter_index)
                if r_value > r_value_threshold:
                    good_stiffness_array.append(slope)
                    good_height_array.append(height_array[-1])
                    diameter_index = ((bush * 9) - 9) + ((branch * 3) - 3) + (trial - 1)
                    good_diameter_array.append(diameter_array[diameter_index])
                
                if make_filtered_plot: 
                    filtered_var_name = 'filtered_fig' + str(bush) + str(branch) + str(trial)
                    filtered_var_name = plt.figure()
                    # Plot the offset displacement and filtered force data for points that had a force reading greater than 20 gf
                    plt.plot(motor_steps, load_cell_readings, marker=marker_type, linestyle='')
                    plt.title(f'Filtered Force vs. Displacement for bush {bush}, branch {branch},trial {trial}')
                    plt.xlabel('Displacement (mm)')
                    plt.ylabel('Force (mN)')
                    plt.xlim(displacement_axis)
                    plt.ylim(filtered_force_axis)
                    plt.grid(True)
                    plt.plot(motor_steps, line, color='red', linestyle='--', label='Regression Line')

                    # Annotate the plot with slope and R-squared value
                    plt.text(50, 11000, f'Slope/Stiffness: {slope:.2f}', fontsize=12, color='blue')
                    plt.text(50, 10000, f'R-squared: {r_value**2:.2f}', fontsize=12, color='blue')
                    plt.text(50, 9000, f'Delta x: {delta_x:.2f} mm', fontsize=12, color='blue')
                    plt.text(50, 8000, f'Delta F: {round(delta_F,2)} mN', fontsize=12, color='blue')
                    plt.text(50, 7000, f'Max F: {max_F:.2f} mN', fontsize=12, color='blue')


                    #plt.show()

                # Perform linear regression
                slope, intercept, r_value, p_value, std_err = linregress(all_motor_steps, all_load_cell_readings)
                line = slope * np.array(all_motor_steps) + intercept
                

                
                if make_unfiltered_plot:
                    unfiltered_var_name = 'unfiltered_fig' + str(bush) + str(branch) + str(trial)
                    unfiltered_var_name = plt.figure()
                    # Plot the unfiltered data without linear regression
                    plt.plot(all_motor_steps, all_load_cell_readings, marker=marker_type, linestyle='')
                    plt.title(f'Unfiltered Force vs. Displacement for bush {bush}, branch {branch},trial {trial}')
                    plt.xlabel('Displacement (mm)')
                    plt.ylabel('Force (mN)')
                    plt.xlim(displacement_axis)
                    plt.ylim(unfiltered_force_axis)
                    plt.grid(True)

                    # Annotate the plot with slope and R-squared value
                    plt.text(30, 11000, f'Slope/Stiffness: {slope:.2f}', fontsize=12, color='blue')
                    plt.text(30, 10000, f'R-squared: {r_value**2:.2f}', fontsize=12, color='blue')
                    plt.plot(all_motor_steps, line, color='red', linestyle='--', label='Regression Line')
                    
                    #plt.show()
            else:
                stiffness_array.append(0)
                delta_x_array.append(0)
                delta_F_array.append(0)
                max_F_array.append(0)

                if make_filtered_plot: 
                    filtered_var_name = 'filtered_fig' + str(bush) + str(branch) + str(trial)
                    filtered_var_name = plt.figure()
                    # Plot the offset displacement and filtered force data for points that had a force reading less than 20 gf
                    plt.plot([0, 0], marker=marker_type, linestyle='')
                    plt.title(f'No force data greater than 180 mN for bush {bush}, branch {branch},trial {trial}')
                
                # Perform linear regression
                slope, intercept, r_value, p_value, std_err = linregress(all_motor_steps, all_load_cell_readings)
                line = slope * np.array(all_motor_steps) + intercept
                                
                if make_unfiltered_plot:
                    unfiltered_var_name = 'unfiltered_fig' + str(bush) + str(branch) + str(trial)
                    unfiltered_var_name = plt.figure()
                    # Plot the unfiltered data without linear regression
                    plt.plot(all_motor_steps, all_load_cell_readings, marker=marker_type, linestyle='')
                    plt.title(f'Unfiltered Force vs. Displacement for bush {bush}, branch {branch},trial {trial}')
                    plt.xlabel('Displacement (mm)')
                    plt.ylabel('Force (mN)')
                    plt.xlim(displacement_axis)
                    plt.ylim(unfiltered_force_axis)
                    plt.grid(True)

                    # Annotate the plot with slope and R-squared value
                    plt.text(30, 11000, f'Slope/Stiffness: {slope:.2f}', fontsize=12, color='blue')
                    plt.text(30, 10000, f'R-squared: {r_value**2:.2f}', fontsize=12, color='blue')
                    plt.plot(all_motor_steps, line, color='red', linestyle='--', label='Regression Line')

                    #plt.show()


    fig=plt.figure()
    ax=fig.add_subplot(1,1,1)
    i = (bush * 9)-9
    x = 0.1

    plt.text(x,0.9,f'Branch 1, Trial 1, stiffness: {round(stiffness_array[i],2)} mN/mm, diameter: {diameter_array[i]} mm, height: {int(height_array[i])} mm',horizontalalignment='left',verticalalignment='center',transform = ax.transAxes)
    plt.text(x,0.85,f'delta x: {round(delta_x_array[i],2)} mm, delta F: {delta_F_array[i]} mN/mm, max Force: {int(max_F_array[i])} mm',horizontalalignment='left',verticalalignment='center',transform = ax.transAxes)
    
    plt.text(x,0.8,f'Branch 1, Trial 2, stiffness: {round(stiffness_array[i+1],2)} mN/mm, diameter: {diameter_array[i+1]} mm, height: {int(height_array[i+1])} mm',horizontalalignment='left',verticalalignment='center',transform = ax.transAxes)
    plt.text(x,0.75,f'delta x: {round(delta_x_array[i+1],2)} mm, delta F: {delta_F_array[i+1]} mN/mm, max Force: {int(max_F_array[i+1])} mm',horizontalalignment='left',verticalalignment='center',transform = ax.transAxes)

    plt.text(x,0.7,f'Branch 1, Trial 3, stiffness: {round(stiffness_array[i+2],2)} mN/mm, diameter: {diameter_array[i+2]} mm, height: {int(height_array[i+2])} mm',horizontalalignment='left',verticalalignment='center',transform = ax.transAxes)
    plt.text(x,0.65,f'delta x: {round(delta_x_array[i+2],2)} mm, delta F: {delta_F_array[i+2]} mN/mm, max Force: {int(max_F_array[i+2])} mm',horizontalalignment='left',verticalalignment='center',transform = ax.transAxes)

    plt.text(x,0.6,f'Branch 2, Trial 1, stiffness: {round(stiffness_array[i+3],2)} mN/mm, diameter: {diameter_array[i+3]} mm, height: {int(height_array[i+3])} mm',horizontalalignment='left',verticalalignment='center',transform = ax.transAxes)
    plt.text(x,0.55,f'delta x: {round(delta_x_array[i+3],2)} mm, delta F: {delta_F_array[i+3]} mN/mm, max Force: {int(max_F_array[i+3])} mm',horizontalalignment='left',verticalalignment='center',transform = ax.transAxes)

    plt.text(x,0.5,f'Branch 2, Trial 2, stiffness: {round(stiffness_array[i+4],2)} mN/mm, diameter: {diameter_array[i+4]} mm, height: {int(height_array[i+4])} mm',horizontalalignment='left',verticalalignment='center',transform = ax.transAxes)
    plt.text(x,0.45,f'delta x: {round(delta_x_array[i+4],2)} mm, delta F: {delta_F_array[i+4]} mN/mm, max Force: {int(max_F_array[i+4])} mm',horizontalalignment='left',verticalalignment='center',transform = ax.transAxes)

    plt.text(x,0.4,f'Branch 2, Trial 3, stiffness: {round(stiffness_array[i+5],2)} mN/mm, diameter: {diameter_array[i+5]} mm, height: {int(height_array[i+5])} mm',horizontalalignment='left',verticalalignment='center',transform = ax.transAxes)
    plt.text(x,0.35,f'delta x: {round(delta_x_array[i+5],2)} mm, delta F: {delta_F_array[i+5]} mN/mm, max Force: {int(max_F_array[i+5])} mm',horizontalalignment='left',verticalalignment='center',transform = ax.transAxes)

    plt.text(x,0.3,f'Branch 3, Trial 1, stiffness: {round(stiffness_array[i+6],2)} mN/mm, diameter: {diameter_array[i+6]} mm, height: {int(height_array[i+6])} mm',horizontalalignment='left',verticalalignment='center',transform = ax.transAxes)
    plt.text(x,0.25,f'delta x: {round(delta_x_array[i+6],2)} mm, delta F: {delta_F_array[i+6]} mN/mm, max Force: {int(max_F_array[i+6])} mm',horizontalalignment='left',verticalalignment='center',transform = ax.transAxes)

    plt.text(x,0.2,f'Branch 3, Trial 2, stiffness: {round(stiffness_array[i+7],2)} mN/mm, diameter: {diameter_array[i+7]} mm, height: {int(height_array[i+7])} mm',horizontalalignment='left',verticalalignment='center',transform = ax.transAxes)
    plt.text(x,0.15,f'delta x: {round(delta_x_array[i+7],2)} mm, delta F: {delta_F_array[i+7]} mN/mm, max Force: {int(max_F_array[i+7])} mm',horizontalalignment='left',verticalalignment='center',transform = ax.transAxes)

    plt.text(x,0.1,f'Branch 3, Trial 3, stiffness: {round(stiffness_array[i+8],2)} mN/mm, diameter: {diameter_array[i+8]} mm, height: {int(height_array[i+8])} mm',horizontalalignment='left',verticalalignment='center',transform = ax.transAxes)
    plt.text(x,0.05,f'delta x: {round(delta_x_array[i+8],2)} mm, delta F: {delta_F_array[i+8]} mN/mm, max Force: {int(max_F_array[i+8])} mm',horizontalalignment='left',verticalalignment='center',transform = ax.transAxes)




fig1000 = plt.figure()
# Perform linear regression for all points
slope, intercept, r_value, p_value, std_err = linregress(height_array, stiffness_array)
line = slope * np.array(height_array) + intercept
# Perform linear regression for good points
good_slope, good_intercept, good_r_value, good_p_value, good_std_err = linregress(good_height_array, good_stiffness_array)
good_line = good_slope * np.array(good_height_array) + good_intercept
# Scatterplot
plt.plot(height_array, stiffness_array, marker=marker_type, linestyle='', color='blue')
plt.plot(bad_height_array, bad_stiffness_array, marker=marker_type, linestyle='', color='red')
plt.title('Stiffness (mN/mm) vs. Height (mm)')
plt.xlabel('Height (mm)')
plt.ylabel('Stiffness (mN/mm)')
plt.xlim(height_axis)
plt.ylim(stiffness_axis)
plt.grid(True)
#plt.show()

# Annotate the plot with slope and R-squared value for all values
plt.text(1000, 1300, f'Slope: {slope:.2f}', fontsize=12, color='red')
plt.text(1000, 1200, f'R-squared: {r_value**2:.2f}', fontsize=12, color='red')
plt.plot(height_array, line, color='red', linestyle='--', label='Regression Line for all data')

if show_good_slope:
    # Annotate the plot with slope and R-squared value for good values
    plt.text(1000, 1100, f'Slope: {good_slope:.2f}', fontsize=12, color='green')
    plt.text(1000, 1000, f'R-squared: {good_r_value**2:.2f}', fontsize=12, color='green')
    plt.plot(good_height_array, good_line, color='green', linestyle='--', label='Regression Line for good points')
                        

fig1001 = plt.figure()
# Perform linear regression for all values
slope, intercept, r_value, p_value, std_err = linregress(diameter_array, stiffness_array)
line = slope * np.array(diameter_array) + intercept
# Perform linear regression for good points
good_slope, good_intercept, good_r_value, good_p_value, good_std_err = linregress(good_diameter_array, good_stiffness_array)
good_line = good_slope * np.array(good_diameter_array) + good_intercept

# Scatterplot
plt.plot(diameter_array, stiffness_array, marker=marker_type, linestyle='', color='blue')
plt.plot(bad_diameter_array, bad_stiffness_array, marker=marker_type, linestyle='', color='red')
plt.title('Stiffness (mN/mm) vs. Diameter (mm)')
plt.xlabel('Diameter (mm)')
plt.ylabel('Stiffness (mN/mm)')
plt.xlim(diameter_axis)
plt.ylim(stiffness_axis)
plt.grid(True)
#plt.show()
# Annotate the plot with slope and R-squared value for all values
plt.text(15, 1300, f'Slope: {slope:.2f}', fontsize=12, color='red')
plt.text(15, 1200, f'R-squared: {r_value**2:.2f}', fontsize=12, color='red')
plt.plot(diameter_array, line, color='red', linestyle='--', label='Regression Line')
  
if show_good_slope:     
    # Annotate the plot with slope and R-squared value for good values
    plt.text(15, 1100, f'Slope: {good_slope:.2f}', fontsize=12, color='green')
    plt.text(15, 1000, f'R-squared: {good_r_value**2:.2f}', fontsize=12, color='green')
    plt.plot(good_diameter_array, good_line, color='green', linestyle='--', label='Regression Line for good points')



fig1003 = plt.figure()

# Sort the stiffnesses from lowest to highest
sorted_stiffness_array = np.array(np.sort(stiffness_array))

# Split the stiffnesses into 3 clusters by length of the list
threshold_1 = 17 
threshold_2 = 35 
low_stiffness_array = sorted_stiffness_array[0:threshold_1]
medium_stiffness_array = sorted_stiffness_array[threshold_1-1:threshold_2]
high_stiffness_array = sorted_stiffness_array[threshold_2:]

# Histogram
#plt.hist(stiffness_array, bins=bin_edges_stiffness)
plt.hist(low_stiffness_array, bins=bin_edges_stiffness)
plt.hist(medium_stiffness_array, bins=bin_edges_stiffness)
plt.hist(high_stiffness_array, bins=bin_edges_stiffness)
plt.title('Stiffness Histogram split by number of stiffness values')
plt.xlabel('Stiffness (mN/mm)')
plt.ylabel('Frequency')

plt.tight_layout()
#plt.show()

fig1010 = plt.figure()
# Histogram
#plt.hist(stiffness_array, bins=bin_edges_stiffness)
plt.hist(stiffness_array, bins=bin_edges_stiffness)
plt.title('Stiffness Histogram')
plt.xlabel('Stiffness (mN/mm)')
plt.ylabel('Frequency')

plt.tight_layout()
#plt.show()


fig1004 = plt.figure()
# Split the stiffnesses into 3 clusters by value
avg = np.mean(stiffness_array)
threshold_1 = 0.5 * avg
threshold_2 = 1.4 * avg


low_indices = []
medium_indices = []
high_indices = []

low_stiffness_array =  []
low_stiffness_heights =[]
low_stiffness_diameters = []

medium_stiffness_array = []
medium_stiffness_heights = []
medium_stiffness_diameters = []

high_stiffness_array = []
high_stiffness_heights = []
high_stiffness_diameters = []



for i in range(len(stiffness_array)):
    if stiffness_array[i] <= threshold_1:
        low_stiffness_array.append(stiffness_array[i])
        low_stiffness_heights.append(height_array[i])
        low_stiffness_diameters.append(diameter_array[i])
        low_indices.append(i)
    elif threshold_1 < stiffness_array[i] <= threshold_2:
        medium_stiffness_array.append(stiffness_array[i])
        medium_stiffness_heights.append(height_array[i])
        medium_stiffness_diameters.append(diameter_array[i])
        medium_indices.append(i)
    else:
        high_stiffness_array.append(stiffness_array[i])
        high_stiffness_heights.append(height_array[i])
        high_stiffness_diameters.append(diameter_array[i])
        high_indices.append(i)
        
# Histogram
#plt.hist(stiffness_array, bins=bin_edges_stiffness)
plt.hist(low_stiffness_array, bins=bin_edges_stiffness)
plt.hist(medium_stiffness_array, bins=bin_edges_stiffness)
plt.hist(high_stiffness_array, bins=bin_edges_stiffness)
plt.axvline(avg, color='k', linestyle='dashed', linewidth=1)
plt.title('Stiffness Histogram split by stiffness values')
plt.xlabel('Stiffness (mN/mm)')
plt.ylabel('Frequency')
plt.text(400, 8, f'Blue = values <= 0.5 * average', fontsize=10, color='blue')
plt.text(400, 7, f'Orange = values > 0.5 * average and <= 1.4 * average', fontsize=10, color='orange')
plt.text(400, 6, f'Green = values > 1.4 * average', fontsize=10, color='green')
plt.text(400, 4, f'---- = Average Value', fontsize=10, color='black')

plt.tight_layout()
#plt.show()


fig1005 = plt.figure()
# Perform linear regression for all points
slope, intercept, r_value, p_value, std_err = linregress(height_array, stiffness_array)
line = slope * np.array(height_array) + intercept
# Perform linear regression for good points
good_slope, good_intercept, good_r_value, good_p_value, good_std_err = linregress(good_height_array, good_stiffness_array)
good_line = good_slope * np.array(good_height_array) + good_intercept
# Scatterplot
plt.plot(low_stiffness_heights, low_stiffness_array, marker=marker_type, linestyle='', color='blue')
plt.plot(medium_stiffness_heights, medium_stiffness_array, marker=marker_type, linestyle='', color='orange')
plt.plot(high_stiffness_heights, high_stiffness_array, marker=marker_type, linestyle='', color='green')
plt.plot(bad_height_array, bad_stiffness_array, marker='$X$', linestyle='', color='red')
plt.title('Stiffness (mN/mm) vs. Height (mm)')
plt.xlabel('Height (mm)')
plt.ylabel('Stiffness (mN/mm)')
plt.xlim(height_axis)
plt.ylim(stiffness_axis)
plt.grid(True)
#plt.show()

# Annotate the plot with slope and R-squared value for all values
plt.text(700, 900, f'Slope: {slope:.2f}', fontsize=12, color='gray')
plt.text(700, 800, f'R-squared: {r_value**2:.2f}', fontsize=12, color='gray')
plt.plot(height_array, line, color='gray', linestyle='--', label='Regression Line for all data')

plt.text(700, 1300, f'Blue = values <= 0.5 * average', fontsize=10, color='blue')
plt.text(700, 1200, f'Orange = values > 0.5 * average and <= 1.4 * average', fontsize=10, color='orange')
plt.text(700, 1100, f'Green = values > 1.4 * average', fontsize=10, color='green')

if show_good_slope:
    # Annotate the plot with slope and R-squared value for good values
    plt.text(1000, 1100, f'Slope: {good_slope:.2f}', fontsize=12, color='green')
    plt.text(1000, 1000, f'R-squared: {good_r_value**2:.2f}', fontsize=12, color='green')
    plt.plot(good_height_array, good_line, color='green', linestyle='--', label='Regression Line for good points')
    

fig1006 = plt.figure()
# Perform linear regression for all values
slope, intercept, r_value, p_value, std_err = linregress(diameter_array, stiffness_array)
line = slope * np.array(diameter_array) + intercept
# Perform linear regression for good points
good_slope, good_intercept, good_r_value, good_p_value, good_std_err = linregress(good_diameter_array, good_stiffness_array)
good_line = good_slope * np.array(good_diameter_array) + good_intercept

# Scatterplot
plt.plot(low_stiffness_diameters, low_stiffness_array, marker=marker_type, linestyle='', color='blue')
plt.plot(medium_stiffness_diameters, medium_stiffness_array, marker=marker_type, linestyle='', color='orange')
plt.plot(high_stiffness_diameters, high_stiffness_array, marker=marker_type, linestyle='', color='green')
plt.plot(bad_diameter_array, bad_stiffness_array, marker='$X$', linestyle='', color='red')
plt.title('Stiffness (mN/mm) vs. Diameter (mm)')
plt.xlabel('Diameter (mm)')
plt.ylabel('Stiffness (mN/mm)')
plt.xlim(diameter_axis)
plt.ylim(stiffness_axis)
plt.grid(True)
#plt.show()
# Annotate the plot with slope and R-squared value for all values
plt.text(0.5, 900, f'Slope: {slope:.2f}', fontsize=12, color='gray')
plt.text(0.5, 800, f'R-squared: {r_value**2:.2f}', fontsize=12, color='gray')
plt.plot(diameter_array, line, color='gray', linestyle='-', label='Regression Line')

plt.text(0.5, 1300, f'Blue = values <= 0.5 * average', fontsize=10, color='blue')
plt.text(0.5, 1200, f'Orange = values > 0.5 * average and <= 1.4 * average', fontsize=10, color='orange')
plt.text(0.5, 1100, f'Green = values > 1.4 * average', fontsize=10, color='green')
  
if show_good_slope:     
    # Annotate the plot with slope and R-squared value for good values
    plt.text(15, 1100, f'Slope: {good_slope:.2f}', fontsize=12, color='green')
    plt.text(15, 1000, f'R-squared: {good_r_value**2:.2f}', fontsize=12, color='green')
    plt.plot(good_diameter_array, good_line, color='green', linestyle='-', label='Regression Line for good points')
      
          
fig1002 = plt.figure()
# Normalize diameters and heights and invert the heights by raising them to the negative one power
normalized_stiffness_array = [(i - min(stiffness_array))/(max(stiffness_array) - min(stiffness_array)) for i in stiffness_array]
normalized_diameter_array = [(i - min(diameter_array))/(max(diameter_array) - min(diameter_array)) for i in diameter_array]
inverted_height_array = [i**(-1) for i in height_array]
normalized_inverted_height_array = [(i - min(inverted_height_array))/(max(inverted_height_array) - min(inverted_height_array)) for i in inverted_height_array]

normalized_bad_diameter_array = [(i - min(diameter_array))/(max(diameter_array) - min(diameter_array)) for i in bad_diameter_array]
normalized_bad_stiffness_array = [(i - min(stiffness_array))/(max(stiffness_array) - min(stiffness_array)) for i in bad_stiffness_array]
inverted_bad_height_array = [i**(-1) for i in bad_height_array]
normalized_inverted_bad_height_array = [(i - min(inverted_height_array))/(max(inverted_height_array) - min(inverted_height_array)) for i in inverted_bad_height_array]

# Perform linear regression for all values
slope, intercept, r_value, p_value, std_err = linregress(normalized_diameter_array, normalized_inverted_height_array)
line = slope * np.array(normalized_diameter_array) + intercept

# Scatterplot
plt.plot(normalized_diameter_array, normalized_inverted_height_array, marker=marker_type, linestyle='', color='blue')
plt.plot(normalized_bad_diameter_array, normalized_inverted_bad_height_array, marker='$X$', linestyle='', color='red')
plt.title(f'Diameter (mm) vs 1/Height (1/mm)')
plt.xlabel('1/Height (1/mm)')
plt.ylabel('Diameter (mm)')
plt.xlim([-0.1,1.1])
plt.ylim([-0.1,1.3])
plt.grid(True)
#plt.show()
# Annotate the plot with slope and R-squared value for all values
plt.text(0.8, 1, f'Slope: {slope:.2f}', fontsize=12, color='red')
plt.text(0.8, 0.9, f'R-squared: {r_value**2:.2f}', fontsize=12, color='red')
plt.plot(normalized_diameter_array, line, color='red', linestyle='--', label='Regression Line')
                
fig1007 = plt.figure()
# To get normalized low, medium, and high values look through matrices with all normalized values then pick out values at the indices in the bad_index_array
normalized_bad_diameter_array = [normalized_diameter_array[i] for i in bad_index_array]
normalized_low_stiffness_diameters = [normalized_diameter_array[i] for i in low_indices]
normalized_inverted_low_stiffness_heights = [normalized_inverted_height_array[i] for i in low_indices]
normalized_medium_stiffness_diameters = [normalized_diameter_array[i] for i in medium_indices]
normalized_inverted_medium_stiffness_heights = [normalized_inverted_height_array[i] for i in medium_indices]
normalized_high_stiffness_diameters = [normalized_diameter_array[i] for i in high_indices]
normalized_inverted_high_stiffness_heights = [normalized_inverted_height_array[i] for i in high_indices]

# Perform linear regression for all values
slope, intercept, r_value, p_value, std_err = linregress(normalized_diameter_array, normalized_inverted_height_array)
line = slope * np.array(normalized_diameter_array) + intercept

# Scatterplot
plt.plot(normalized_low_stiffness_diameters, normalized_inverted_low_stiffness_heights, marker=marker_type, linestyle='', color='blue')
plt.plot(normalized_medium_stiffness_diameters, normalized_inverted_medium_stiffness_heights, marker=marker_type, linestyle='', color='orange')
plt.plot(normalized_high_stiffness_diameters, normalized_inverted_high_stiffness_heights, marker=marker_type, linestyle='', color='green')
plt.plot(normalized_bad_diameter_array, normalized_inverted_bad_height_array, marker='$X$', linestyle='', color='red')

plt.title(f'Diameter (mm) vs 1/Height (1/mm)')
plt.xlabel('1/Height (1/mm)')
plt.ylabel('Diameter (mm)')
plt.xlim([-0.1,1.1])
plt.ylim([-0.1,1.3])
plt.grid(True)

plt.text(0, 1.2, f'Blue = values <= 0.5 * average', fontsize=10, color='blue')
plt.text(0, 1.1, f'Orange = values > 0.5 * average and <= 1.4 * average', fontsize=10, color='orange')
plt.text(0, 1, f'Green = values > 1.4 * average', fontsize=10, color='green')
#plt.show()

fig1008 = plt.figure()
# To get normalized low, medium, and high values look through matrices with all normalized values then pick out values at the indices in the bad_index_array
normalized_bad_diameter_array = [normalized_diameter_array[i] for i in bad_index_array]

normalized_low_stiffness_diameters = [normalized_diameter_array[i] for i in low_indices]
normalized_inverted_low_stiffness_heights = [normalized_inverted_height_array[i] for i in low_indices]
normalized_low_stiffness_array = [normalized_stiffness_array[i] for i in low_indices]

normalized_medium_stiffness_diameters = [normalized_diameter_array[i] for i in medium_indices]
normalized_inverted_medium_stiffness_heights = [normalized_inverted_height_array[i] for i in medium_indices]
normalized_medium_stiffness_array = [normalized_stiffness_array[i] for i in medium_indices]

normalized_high_stiffness_diameters = [normalized_diameter_array[i] for i in high_indices]
normalized_inverted_high_stiffness_heights = [normalized_inverted_height_array[i] for i in high_indices]
normalized_high_stiffness_array = [normalized_stiffness_array[i] for i in high_indices]

# Perform linear regression for all values
slope, intercept, r_value, p_value, std_err = linregress(normalized_diameter_array, normalized_stiffness_array)
line = slope * np.array(normalized_diameter_array) + intercept

# Scatterplot
plt.plot(normalized_low_stiffness_diameters, normalized_low_stiffness_array, marker=marker_type, linestyle='', color='blue')
plt.plot(normalized_medium_stiffness_diameters, normalized_medium_stiffness_array, marker=marker_type, linestyle='', color='orange')
plt.plot(normalized_high_stiffness_diameters, normalized_high_stiffness_array, marker=marker_type, linestyle='', color='green')
plt.plot(normalized_bad_diameter_array, normalized_bad_stiffness_array, marker='$X$', linestyle='', color='red')

plt.title(f'Diameter (mm) vs Stiffness (mN/mm)')
plt.xlabel('Diameter (mm)')
plt.ylabel('Stiffness (mN/mm)')
plt.xlim([-0.1,1.1])
plt.ylim([-0.1,1.1])
plt.grid(True)
#plt.show()
# Annotate the plot with slope and R-squared value for all values
plt.text(0.8, 1, f'Slope: {slope:.2f}', fontsize=12, color='gray')
plt.text(0.8, 0.9, f'R-squared: {r_value**2:.2f}', fontsize=12, color='gray')
plt.plot(normalized_diameter_array, line, color='gray', linestyle='-', label='Regression Line')
plt.text(0, 1, f'Blue = values <= 0.5 * average', fontsize=10, color='blue')
plt.text(0, 0.9, f'Orange = values > 0.5 * average and <= 1.4 * average', fontsize=10, color='orange')
plt.text(0, 0.8, f'Green = values > 1.4 * average', fontsize=10, color='green')

fig1009 = plt.figure()
# Perform linear regression for all values
slope, intercept, r_value, p_value, std_err = linregress(normalized_inverted_height_array, normalized_stiffness_array)
line = slope * np.array(normalized_inverted_height_array) + intercept

# Scatterplot
plt.plot(normalized_inverted_low_stiffness_heights, normalized_low_stiffness_array, marker=marker_type, linestyle='', color='blue')
plt.plot(normalized_inverted_medium_stiffness_heights, normalized_medium_stiffness_array, marker=marker_type, linestyle='', color='orange')
plt.plot(normalized_inverted_high_stiffness_heights, normalized_high_stiffness_array, marker=marker_type, linestyle='', color='green')
plt.plot(normalized_inverted_bad_height_array, normalized_bad_stiffness_array, marker='$X$', linestyle='', color='red')

plt.title(f'1/Height (1/mm) vs Stiffness (mN/mm)')
plt.xlabel('1/Height (1/mm)')
plt.ylabel('Stiffness (mN/mm)')
plt.xlim([-0.1,1.1])
plt.ylim([-0.1,1.1])
plt.grid(True)
#plt.show()
# Annotate the plot with slope and R-squared value for all values
plt.text(0.8, 1, f'Slope: {slope:.2f}', fontsize=12, color='gray')
plt.text(0.8, 0.9, f'R-squared: {r_value**2:.2f}', fontsize=12, color='gray')
plt.plot(normalized_inverted_height_array, line, color='gray', linestyle='-', label='Regression Line')
plt.text(0, 1, f'Blue = values <= 0.5 * average', fontsize=10, color='blue')
plt.text(0, 0.9, f'Orange = values > 0.5 * average and <= 1.4 * average', fontsize=10, color='orange')
plt.text(0, 0.8, f'Green = values > 1.4 * average', fontsize=10, color='green')



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
  
  
print(len(bad_diameter_array))
print(len(bad_height_array))
print(len(bad_stiffness_array))

print(max(load_cell_readings))
print(max(stiffness_array))
