#!/usr/bin/env python3

# arduino_interface.py
#
# Chelse VanAtter and Jostan Brown
#
# Code to send commands to the Arduino to move the robot and collect sensor data that the Arduino is outputting

# Import ROS2 stuff
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty, Trigger, SetBool

# Import other necessary libraries
import serial
import selectors

# This class interfaces with the Arduino
class ArduinoInterface(Node):
    # This function initializes variables like the serial port and baud rate
    def __init__(self):
        super().__init__('arduino_interface', namespace='blueberry_dc')
        
        # Define Arduino serial port and baud rate
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 57600)

        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value

        self.get_logger().info(f"Using serial port: {serial_port} and baud rate: {baud_rate}")
        

        # Open the serial connection to Arduino
        self.arduino_connection = serial.Serial(serial_port, baud_rate, timeout=2)

        self.selector = selectors.DefaultSelector()  # Create a selector object
        
        # Register the serial connection for monitoring read events
        self.selector.register(self.arduino_connection, selectors.EVENT_READ)

        self.data_publisher = self.create_publisher(Float64MultiArray, 'load_cell_data', 10)
        
        # Create services for all of the motor commands
        self.up_command_srv = self.create_service(Trigger, 'move_up', self.up_command_callback)
        self.down_command_srv = self.create_service(Trigger, 'move_down', self.down_command_callback)
        self.in_command_srv = self.create_service(Trigger, 'move_in', self.in_command_callback)
        self.out_command_srv = self.create_service(Trigger, 'move_out', self.out_command_callback)
        self.run_trial_srv = self.create_service(Trigger, 'run_trial', self.run_trial_callback)
    
    # This function publishes the message data                
    def run(self):
        while rclpy.ok():
            events = self.selector.select(timeout=0.1)
            for key, mask in events:
                if mask & selectors.EVENT_READ:
                    received_data = key.fileobj.readline().decode().strip()
                    try:
                        data = [float(x) for x in received_data.split(', ')]
                    except ValueError or UnicodeDecodeError:
                        continue
                    data.append(float(2))
                    data.append(float(2))
                    
                    msg = Float64MultiArray()
                    msg.data = data
                    
                    self.data_publisher.publish(msg)
                    self.get_logger().info('Published data.')
            
            rclpy.spin_once(self, timeout_sec=0)

    # This function sends/writes commands to the Arduino 
    def send_command(self, command):
        self.arduino_connection.write(command.encode())
        self.get_logger().info(f"Sent command: {command}") 

    # This function sends the up command when the service is called
    def up_command_callback(self, request, response):
        self.get_logger().info('Received up command request')
        self.send_command('U')
        response.success = True
        response.message = 'Up command executed'
        return response
    
    # This function sends the down command when the service is called
    def down_command_callback(self, request, response):
        self.get_logger().info('Received down command request')
        self.send_command('D')
        response.success = True
        response.message = 'Down command executed'
        return response
    
    # This function sends the in command when the service is called
    def in_command_callback(self, request, response):
        self.get_logger().info('Received in command request')
        self.send_command('I')
        response.success = True
        response.message = 'In command executed'
        return response
    
    # This function sends the out command when the service is called
    def out_command_callback(self, request, response):
        self.get_logger().info('Received out command request')
        self.send_command('O')
        response.success = True
        response.message = 'Out command executed'
        return response 
    
    # This function sends the run trial command when the service is called
    def run_trial_callback(self, request, response):
        self.get_logger().info('Received run trial request')
        self.send_command('C')
        response.success = True
        response.message = 'Trial started'
        return response

# This is the main function which initializes the node, starts it, then destroys it    
def main(args=None):
    rclpy.init(args=args)
    arduino_interface = ArduinoInterface()
    arduino_interface.run()
    rclpy.spin(arduino_interface)
    arduino_interface.destroy_node()

if __name__ == "__main__":
    main()
