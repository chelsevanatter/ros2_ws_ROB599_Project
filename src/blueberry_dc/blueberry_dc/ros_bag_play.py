import rclpy
from rclpy.node import Node
import subprocess
from rclpy.parameter import Parameter

class BagPlayerNode(Node):
    def __init__(self):
        super().__init__('ros_bag_play')
        bush = self.declare_parameter('bush', 0).value
        branch = self.declare_parameter('branch', 0).value
        trial = self.declare_parameter('trial', 0).value

        # Construct the path to the bag file based on the provided bush_number, branch_number, and trial_number
        self.bag_file_path = f'/home/chelse/ros2_ws_ROB599_Project/bag_files/bush_{bush}_branch_{branch}_trial_{trial}/bush_{bush}_branch_{branch}_trial_{trial}.db3'

        # Create a timer to start playing the bag file 5 seconds after the node starts
        self.timer = self.create_timer(1.0, self.play_bag_file)

    def play_bag_file(self):
        # Run the 'ros2 bag play' command to play the bag file with increased speed
        playback_command = f'ros2 bag play -r 100.0 {self.bag_file_path}'
        subprocess.Popen(playback_command, shell=True)
        
        # Stop the ROS 2 node after playing the bag file
        self.get_logger().info("Bag file playback complete. Stopping the node.")
        self.timer.cancel()  # Cancel the timer to prevent any further executions of play_bag_file
        rclpy.shutdown()     # Shutdown the ROS 2 node

def main(args=None):
    rclpy.init(args=args)
    bag_player_node = BagPlayerNode()
    rclpy.spin(bag_player_node)
    bag_player_node.destroy_node()

if __name__ == '__main__':
    main()
