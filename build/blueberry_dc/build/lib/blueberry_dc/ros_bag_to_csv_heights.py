import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import csv
import os
from rclpy.parameter import Parameter

class DataSubscriber(Node):
    def __init__(self):
        super().__init__('ros_bag_to_csv')
        bush = self.declare_parameter('bush', 0).value
        branch = self.declare_parameter('branch', 0).value
        trial = self.declare_parameter('trial', 0).value

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/blueberry_dc/load_cell_data',
            self.data_callback,
            10
        )
        # Construct the CSV file name based on bush_number, branch_number, and trial_number
        self.csv_file = f'/home/chelse/ros2_ws_ROB599_Project/csv_files/bush_{bush}_branch_{branch}_trial_{trial}_height.csv'
        self.file_prefix = f'/home/chelse/ros2_ws_ROB599_Project/bag_files_corrected_heights/bush_{bush}_branch_{branch}_trial_{trial}'
        self.csv_writer = csv.writer(open(self.csv_file, 'w', newline=''))
        self.csv_writer.writerow(['Iteration', 'Time Stamp', 'Load Cell Reading (gf)', 'Displacement (mm)', 'Height (mm)'])
        self.last_message_time = None

    def data_callback(self, msg):
        directory = '/home/chelse/ros2_ws_ROB599_Project/bag_files_corrected_heights'
        height = 0
        
        # Check if the specified directory exists
        if not os.path.isdir(directory):
            print(f"Error: {directory} is not a valid directory.")

        # List all files in the directory
        files = os.listdir(directory)
        
        # Find the file that matches the prefix
        matching_files = [file for file in files if file.startswith(self.file_prefix)]

        # Check if there is exactly one matching file
        if len(matching_files) == 1:
            # Get all characters in file name after the 23rd character
            height = matching_files[0][23:]  # Slicing from the 24th character to the end
        elif len(matching_files) > 1:
            print("Multiple files match the prefix. Please provide a more specific file name.")
        else:
            print("No matching file found.")
                        
        
        data = [msg.data[0], msg.data[1], msg.data[2], msg.data[3],height]
        self.csv_writer.writerow(data)
        self.last_message_time = self.get_clock().now()
        self.get_logger().info('Received message and wrote to CSV file.')
    
def main(args=None):
    rclpy.init(args=args)
    subscriber = DataSubscriber()
    shutdown_timer = subscriber.create_timer(5.0, subscriber.destroy_node)
    rclpy.spin(subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
