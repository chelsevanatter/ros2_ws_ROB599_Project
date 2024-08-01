import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

import csv

class DataSubscriber(Node):
    def __init__(self):
        super().__init__('data_subscriber')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/blueberry_dc/load_cell_data',
            self.data_callback,
            10
        )
        self.csv_file = '/home/chelse/ros2_ws/csv_files/load_cell_data.csv'
        self.csv_writer = csv.writer(open(self.csv_file, 'w', newline=''))
        self.csv_writer.writerow(['Data1', 'Data2', 'Data3', 'Data4', 'Data5', 'Data6'])

    def data_callback(self, msg):
        data = [msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5]]
        self.csv_writer.writerow(data)
        self.get_logger().info('Received message and wrote to CSV file.')

def main(args=None):
    rclpy.init(args=args)
    subscriber = DataSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
