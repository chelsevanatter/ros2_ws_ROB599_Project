import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
import csv
import time
import subprocess
import signal
import os
import glob
import threading
from PyQt6.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QProgressBar, QLabel, QSpinBox, QPlainTextEdit
import sys


class BagRecorder:
    def __init__(self, topics):
        self.topics = topics
        self.process = None

    def start_recording(self, bag_name):
        cmd = ['ros2', 'bag', 'record', '-o', bag_name] + self.topics
        self.process = subprocess.Popen(cmd)
        print(f'Started recording topics: {self.topics} into bag: {bag_name}')
        print("command: ", cmd)

    def stop_recording(self):
        if self.process:
            self.process.send_signal(signal.SIGINT)  # Send SIGINT to mimic Ctrl+C
            self.process.wait()  # Wait for the process to terminate
            print('Stopped recording.')
            
class ROSDataSubscriber(Node):
    def __init__(self):
        super().__init__('data_subscriber')

        self.actuator_height = 430
        self.bag_file_dir = '/home/jostan/bag_files/'
        self.inactivity_timeout = 3.0

        self.load_cell_force = 0
        self.last_load_cell_message_time = None

        self.move_up_client = self.create_client(Trigger, '/blueberry_dc/move_up')
        self.move_down_client = self.create_client(Trigger, '/blueberry_dc/move_down')
        self.move_in_client = self.create_client(Trigger, '/blueberry_dc/move_in')
        self.move_out_client = self.create_client(Trigger, '/blueberry_dc/move_out')
        self.run_trial_client = self.create_client(Trigger, '/blueberry_dc/run_trial')
        while not self.move_up_client.wait_for_service(timeout_sec=1.0):
            # self.get_logger().info('service not available, waiting again...')
            print('service 1 not available, waiting again...')
        while not self.move_down_client.wait_for_service(timeout_sec=1.0):
            # self.get_logger().info('service not available, waiting again...')
            print('service 2 not available, waiting again...')
        while not self.run_trial_client.wait_for_service(timeout_sec=1.0):
            # self.get_logger().info('service not available, waiting again...')
            print('service 3 not available, waiting again...')
        while not self.move_in_client.wait_for_service(timeout_sec=1.0):
            # self.get_logger().info('service not available, waiting again...')
            print('service 4 not available, waiting again...')
        while not self.move_out_client.wait_for_service(timeout_sec=1.0):
            # self.get_logger().info('service not available, waiting again...')
            print('service 5 not available, waiting again...')

        self.bag_recorder = BagRecorder(['/blueberry_dc/load_cell_data', 
                                         '/camera/depth/camera_info',
                                         '/camera/depth/image_rect_raw',
                                         '/camera/color/camera_info',
                                         '/camera/color/image_rect_raw',
                                         '/camera/aligned_depth_to_color/camera_info',
                                         '/camera/aligned_depth_to_color/image_raw',
                                         '/accel0', '/accel1', '/accel2',
                                         '/gyro0', '/gyro1', '/gyro2',
                                         '/orientation0', '/orientation1', '/orientation2',])


        self.subscription = self.create_subscription(Float64MultiArray, '/blueberry_dc/load_cell_data', self.data_callback, 10)



    def data_callback(self, msg):
        self.last_load_cell_message_time = self.get_clock().now()
        self.load_cell_force = msg.data[2]

    def load_cell_publishing(self):
        current_time = self.get_clock().now()
        if self.last_load_cell_message_time is None:
            return False
        elapsed = current_time - self.last_load_cell_message_time
        if elapsed.nanoseconds * 1e-9 > self.inactivity_timeout:
            return False  # No publishing activity
        return True  # Active publishing

    def send_command(self, move_client):
        request = Trigger.Request()
        response = move_client.call_async(request)


class MainWindow(QWidget):
    def __init__(self, ros_node):
        super().__init__()

        self.ros_node = ros_node

        self.initUI()

    def initUI(self):
        layout = QVBoxLayout(self)

        # Directional buttons layout
        up_down_button_layout = QVBoxLayout()
        in_out_button_layout = QHBoxLayout()
        button_layout = QHBoxLayout()
        upButton = QPushButton('↑')
        downButton = QPushButton('↓')
        inButton = QPushButton('←')
        outButton = QPushButton('→')

        upButton.clicked.connect(self.upButtonClicked)
        downButton.clicked.connect(self.downButtonClicked)
        inButton.clicked.connect(self.inButtonClicked)
        outButton.clicked.connect(self.outButtonClicked)

        upButton.setFixedSize(100, 100)
        downButton.setFixedSize(100, 100)
        inButton.setFixedSize(100, 100)
        outButton.setFixedSize(100, 100)

        upButton.setStyleSheet("font: 75 30pt;")
        downButton.setStyleSheet("font: 75 30pt;")
        inButton.setStyleSheet("font: 75 30pt;")
        outButton.setStyleSheet("font: 75 30pt;")

        upButton.setShortcut('w')
        downButton.setShortcut('s')
        inButton.setShortcut('a')
        outButton.setShortcut('d')

        up_down_button_layout.addWidget(upButton)
        up_down_button_layout.addWidget(downButton)
        in_out_button_layout.addWidget(inButton)
        in_out_button_layout.addWidget(outButton)
        button_layout.addLayout(up_down_button_layout)
        button_layout.addLayout(in_out_button_layout)

        layout.addLayout(button_layout)

        # Power bar and label for force sensor
        self.powerBar = QProgressBar()
        self.powerBar.setMinimum(0)
        self.powerBar.setMaximum(1000)
        self.powerBar.setTextVisible(False)
        self.powerBar.setFixedHeight(40)
        self.forceLabel = QLabel("0 N")
        self.forceLabel.setStyleSheet("font: 75 20pt;")
        self.forceLabel.setFixedWidth(90)
        

        power_bar_layout = QHBoxLayout()
        power_bar_layout.addWidget(self.forceLabel)
        power_bar_layout.addWidget(self.powerBar)

        layout.addLayout(power_bar_layout)

        # Start trial button
        startTrialButton = QPushButton('Start Trial')
        startTrialButton.setStyleSheet("font: 75 20pt; text-align: center;")

        startTrialButton.clicked.connect(self.startTrialClicked)
        layout.addWidget(startTrialButton)

        # Add spin boxes for integer inputs
        self.bush_num_box = QSpinBox()
        self.branch_num_box = QSpinBox()
        self.trial_num_box = QSpinBox()

        self.bush_num_box.setMinimum(1)
        self.branch_num_box.setMinimum(1)
        self.trial_num_box.setMinimum(1)

        self.bush_num_box_label = QLabel("Bush Number: ")
        self.branch_num_box_label = QLabel("Branch Number: ")
        self.trial_num_box_label = QLabel("Trial Number: ")

        self.bush_num_box.setFixedHeight(60)
        self.branch_num_box.setFixedHeight(60)
        self.trial_num_box.setFixedHeight(60)

        bush_layout = QHBoxLayout()
        branch_layout = QHBoxLayout()
        trial_layout = QHBoxLayout()

        bush_layout.addWidget(self.bush_num_box_label)
        bush_layout.addWidget(self.bush_num_box)
        branch_layout.addWidget(self.branch_num_box_label)
        branch_layout.addWidget(self.branch_num_box)
        trial_layout.addWidget(self.trial_num_box_label)
        trial_layout.addWidget(self.trial_num_box)

        self.actuator_height_box = QSpinBox()
        self.actuator_height_box.setMinimum(0)
        self.actuator_height_box.setMaximum(1500)
        self.actuator_height_box.setValue(self.ros_node.actuator_height)
        self.actuator_height_box.setFixedHeight(60)
        self.actuator_height_box_label = QLabel("Actuator Height: ")
        actuator_height_layout = QHBoxLayout()

        actuator_height_layout.addWidget(self.actuator_height_box_label)
        actuator_height_layout.addWidget(self.actuator_height_box)

        layout.addLayout(bush_layout)
        layout.addLayout(branch_layout)
        layout.addLayout(trial_layout)
        layout.addLayout(actuator_height_layout)

        # make window to print stuff to in application
        self.log_window = QPlainTextEdit()
        self.log_window.setReadOnly(True)
        layout.addWidget(self.log_window)


        self.setLayout(layout)

    def upButtonClicked(self):
        self.ros_node.send_command(self.ros_node.move_up_client)
        self.actuator_height_box.setValue(self.actuator_height_box.value() + 5)


    def downButtonClicked(self):
        self.ros_node.send_command(self.ros_node.move_down_client)
        self.actuator_height_box.setValue(self.actuator_height_box.value() - 5)

    def inButtonClicked(self):
        self.ros_node.send_command(self.ros_node.move_in_client)

    def outButtonClicked(self):
        self.ros_node.send_command(self.ros_node.move_out_client)

    def startTrialClicked(self):
        bag_name = self.get_file_name()
        if self.check_for_existing_bag(bag_name):
            self.log_window.appendPlainText(f"Bag file exists, delete or rename it and try again. Bag name: {bag_name}")
            return
        
        self.log_window.appendPlainText(f"Starting trial with bag name: {bag_name}")
        self.ros_node.bag_recorder.start_recording(bag_name)
        self.ros_node.send_command(self.ros_node.run_trial_client)
        time.sleep(1)
        while self.ros_node.load_cell_publishing():
            self.updatePowerBar()
            time.sleep(0.1)

        self.ros_node.bag_recorder.stop_recording()
        self.log_window.appendPlainText(f"Trial finished. Bag file saved as: {bag_name}")

    def updatePowerBar(self):
        force_value = int(self.ros_node.load_cell_force)
        self.powerBar.setValue(force_value)
        self.forceLabel.setText(f"{force_value} N")
        # force update of app
        QApplication.processEvents()


    def get_file_name(self):
        bush_num = self.bush_num_box.value()
        branch_num = self.branch_num_box.value()
        trial_num = self.trial_num_box.value()
        actuactor_height = self.actuator_height_box.value()
        bag_name = self.ros_node.bag_file_dir + 'bush_' + str(bush_num) + '_branch_' + str(branch_num) + '_trial_' + str(trial_num) + '_height_' + str(actuactor_height) 
        return bag_name
    
    def check_for_existing_bag(self, bag_name):
        bag_name_to_check_for = bag_name.split('_height')[0] + '*'
        matching_bag_names = glob.glob(bag_name_to_check_for)

        if len(matching_bag_names) > 0:
            return True
        else:
            return False

def run_data_subscriber(ros_node):
    rclpy.spin(ros_node)
    
def main():
    
    rclpy.init()
    ros_node = ROSDataSubscriber()
    
    
    ros_thread = threading.Thread(target=run_data_subscriber, args=(ros_node,))
    ros_thread.start()
    
    app = QApplication(sys.argv)

    # Set the default style for the application
    app.setStyleSheet("""
        * {
            font-size: 16pt;
            font-family: Arial, sans-serif;
        }

        QSpinBox {
            font-size: 16pt; /* Larger text inside the spin box */
        }

        QSpinBox::up-button, QSpinBox::down-button {
            width: 30px; /* Width of the buttons */
            height: 30; /* Height of the buttons */
        }

        QSpinBox::up-arrow, QSpinBox::down-arrow {
            width: 14px; /* Width of the arrows */
            height: 14px; /* Height of the arrows */
        }
    """)

    mainWindow = MainWindow(ros_node)
    mainWindow.show()
    app.exec()

    ros_node.destroy_node()
    rclpy.shutdown()

    ros_thread.join()

if __name__ == '__main__':
    main()
