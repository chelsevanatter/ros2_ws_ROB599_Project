This code is designed for testing the stiffness and damping of blueberry bushes with a test-rig that needs to be set to different contact points (heights) on the plant then collect force and displacement data. 

To run this code you need to colcon build the ros2_ws which has a package called blueberry_dc. 

Before running the code you need to do the following steps in this exact order: 
	1. Plug the Arduino on the Husky robot into your computer
	2. Turn on the IMU modules and plug the main IMU Arduino into your computer
	3. Open a terminal and run the command sudo docker run -it ros:iron
	4. Open a terminal and run the command sudo docker run -it --rm -v /dev:/dev --	privileged --net=host microros/micro-ros-agent:iron serial --dev /dev/ttyACM1 -v6
	5. Open a terminal and enter the ros2_ws then use the command ros2 launch blueberry_dc blueberry_dc.launch
	6. Open a terminal and enter the blueberry_dc/ package in the ros2_ws/src/ directory then use the command python3 data_subscriber_gui.py
	
This will save ros2 bag files to the bag_files directory in the ros2_ws. 

When you run the data_subscriber gui, it will prompt you to move the robot up or down, in or out, and start the trial (push on the blueberry branch) which will then ask the user to input the bush, branch, and trial number then start pushing on a blueberry plant and taking force readings.

This code will save the data to a bag file which needs to be converted to a csv file and if you want to plot the force vs displacement to calculate stiffness. If you have a csv file you can use the command: 
python3 StiffnessPlotter.py and the outputted slope on the graph is the stiffness value in g/mm. 
The first graph filters out all data less than 40g of force because that data is sometimes inconsistent and messy while the second graph shows all of the data. 
