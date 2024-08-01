# Blueberry Dynamics Test-rig

The Blueberry Dynamics Test-rig is for measuring the stiffness and damping of blueberry plants.

## Project Description: 
- A big push in robotics is to have simulated enviroments for testing software. 
- My research aims to make a simulation environemnt with blueberry bushes that have realistic dynamic behaviors and physics. 
- For this I plan to take a real to sim approach where I am collecting real data on bluberry cane (branch) physics and mechanical properties such as stiffness and damping. 
- To collect this data on the stiffness and damping of blueberry canes, Jostan Brown and I have developed a robotic test-rig for pushing on blueberry canes at different 
  heights and colelcting force, displacement, velocity, and RBG-D images to calculate stiffness and damping. 
- The robot platform used is the Husky robot
- Mounted to the Husky was an 80/20 metal frame with wooden shelves. 
- There are two linear actuators mounted to the frame for vertical and horizontal movement.
- The whole horizontal linear actuator is able to rotate which is helpful for aliging the mechanism to push on blueberry branches.
- On the horizontal linear actuator there is an assembly that can rotate up and down to align with how the branches are pointing towards or away from the robot so the force applied is perpendicular to the branches. 
- The assembly consists of a load cell, an intel RealSense 405 RBG-D camera, a buckle mechanism, and different sized interchangable U-shaped probes that buckle in for pushing on different sized blueberry branches. 
- The load cell measures force in grams force and the displacement is measured in millimeters.
- IMU modules created and programmed by Mark Frost were placed on the blueberry branches while the robot pushed on them to get the velocity for calculating damping which is velocity vs displacement. 
  - More details and code for the IMU modules can be found here: https://github.com/markfrosty/Tree-Sensorization-for-Robotic-Fruit-Harvesting/tree/main
- To run this ROS2 package you will need a computer with Ubuntu 22.04 and ROS humble installed.

## Contributors
- Chelse VanAtter: test-rig hardware and software
- Jostan Brown: test-rig hardware and software, Husky navigation software
- Mark Frost: IMU module hardware and software

## Installations
- The only installation you need to run this is for the intel RealSense camera
- Use this command sudo apt install ros-humble-realsense2-*

## Usage
- To run this code you need to colcon build the ros2_ws_ROB599_Project workspace which has a package called blueberry_dc. 

- To run the code you need to do the following steps in this exact order: 
	1. Plug the Arduino Mega on the Husky robot into your computer running Ubuntu 22.04
	2. Turn on each of the three IMU modules and plug the main IMU Arduino into your computer running Ubuntu 22.04
	3. Open a terminal and run the command sudo docker run -it ros:iron
	4. Open a terminal and run the command sudo docker run -it --rm -v /dev:/dev --	privileged --net=host microros/micro-ros-agent:iron serial --dev /dev/ttyACM1 -v6
	5. Open a terminal and enter the ros2_ws then use the command ros2 launch blueberry_dc blueberry_dc.launch
	6. Open a terminal and enter the blueberry_dc/ package in the ros2_ws_ROB599_Project/src/ directory then use the command python3 data_subscriber_gui.py
	
- This will save ros2 bag files to the bag_files directory in the ros2_ws_ROB599_Project directory. 

- When you run the data_subscriber gui, it will prompt you to move the robot up or down, in or out, and start the trial (push on the blueberry branch) which will then ask the user to input the bush, branch, and trial number then start pushing on a blueberry plant and taking force readings.

- This code will save the data to a bag file which needs to be converted to a csv file and if you want to plot the force vs displacement to calculate stiffness. 
- If you have a csv file you can use the command: python3 StiffnessPlotter.py and the outputted slope on the graph is the stiffness value in gf/mm. 
- The first graph filters out all data less than 40g of force because that data is sometimes inconsistent and messy while the second graph shows all of the data. 

## Revision Notes

- I converted units for stiffness mN/mm instead of gf/mm so force was converted from grams force to milli Newtons

## Common Errors and Troubleshooting

- The following commands will solve this error: UserWarning: Unable to import Axes3D. This may be due to multiple versions of Matplotlib being installed (e.g. as a system package and as a pip package). As a result, the 3D projection is not available.  warnings.warn("Unable to import Axes3D. This may be due to multiple versions of "Warning: Ignoring XDG_SESSION_TYPE=wayland on Gnome. Use QT_QPA_PLATFORM=wayland to run on Wayland anyway.
- pip list | grep matplotlib
- apt list --installed | grep matplotlib
- pip uninstall matplotlib
- sudo apt-get remove --purge python3-matplotlib
- pip install matplotlib
- sudo apt-get update
- sudo apt-get upgrade
- sudo apt-get install python3-tk





## License
[BSD 3-Clause] https://opensource.org/license/bsd-3-clause
