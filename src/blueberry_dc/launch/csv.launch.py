# csv.launch.py
#
# Chelse VanAtter
#
# Code to launch the ros_bag_to_csv and ros_bag_play nodes

# Import ROS2 launch stuff
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# This function launches the ros_bag_to_csv and ros_bag_play nodes
def generate_launch_description():

    # Declare launch arguments for bush number, branch number, and trial number
    bush = DeclareLaunchArgument('bush', default_value='0', description='Bush number')
    branch = DeclareLaunchArgument('branch', default_value='0', description='Branch number')
    trial = DeclareLaunchArgument('trial', default_value='0', description='Trial number')

    # Launch the ros_bag_to_csv node
    ros_bag_to_csv_node = Node(
        package='blueberry_dc',  # Replace 'your_package_name' with the actual package name
        executable='ros_bag_to_csv',
        name='ros_bag_to_csv',
        output='screen',
        parameters=[
            {'bush': LaunchConfiguration('bush')},
            {'branch': LaunchConfiguration('branch')},  # Corrected parameter name
            {'trial': LaunchConfiguration('trial')}
        ]
    )

    # Launch the ros_bag_play node
    ros_bag_play_node = Node(
        package='blueberry_dc',  
        executable='ros_bag_play',
        name='ros_bag_play',
        output='screen',
        parameters=[
            {'bush': LaunchConfiguration('bush')},
            {'branch': LaunchConfiguration('branch')},
            {'trial': LaunchConfiguration('trial')}
        ]
    )
    
    return LaunchDescription([
        bush,
        branch,
        trial,
        ros_bag_to_csv_node,
        ros_bag_play_node
    ])
