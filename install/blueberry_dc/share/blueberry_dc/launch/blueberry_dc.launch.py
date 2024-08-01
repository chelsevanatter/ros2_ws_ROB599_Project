# blueberry_dc.launch.py
#
# Chelse VanAtter and Jostan Brown
#
# Code to launch the intel RealSense 405 RGB-D camera and the arduino_interface code

# Import ROS2 launch stuff
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare

# This function launches the realsense launch file and the arduino_interface node
def generate_launch_description():

    realsense_launch =  IncludeLaunchDescription(
                            PythonLaunchDescriptionSource([
                                PathJoinSubstitution([
                                    FindPackageShare('realsense2_camera'),
                                    'launch',
                                    'rs_launch.py'
                                ])    
                            ]),
                            launch_arguments={
                                'depth_module.profile': '848,480,5',
                                'rgb_camera.profile':'848,480,5',
                                'align_depth.enable': 'true',
                            }.items()
                        )                          

    arduino_interface_node = Node(
        package='blueberry_dc',
        executable='arduino_interface',
        name='arduino_interface',
        output='screen'
    )
    
    return LaunchDescription([
        realsense_launch,
        arduino_interface_node,
        
    ])