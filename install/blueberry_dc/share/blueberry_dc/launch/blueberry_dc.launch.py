from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare

# launch the realsense launch file
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
                                  
    # Launch RViz with a specific configuration file
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', '/home/jostan/ros2_ws/src/blueberry_dc/config/rviz_config1.rviz'],
    #     output='screen'
    # )

    arduino_interface_node = Node(
        package='blueberry_dc',
        executable='arduino_interface',
        name='arduino_interface',
        output='screen'
    )
    
    return LaunchDescription([
        realsense_launch,
        # rviz_node,
        arduino_interface_node,
        
    ])