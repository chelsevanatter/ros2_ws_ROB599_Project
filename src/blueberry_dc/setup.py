import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'blueberry_dc'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools', 'rosidl_default_generators'],  # Added rosidl_default_generators
    zip_safe=True,
    maintainer='Chelse VanAtter',
    maintainer_email='chelsevanatter@gmail.com',
    description='Package for controlling two linear actuators with stepper motors, camera, load cell, and IMU modules for blueberry cane stiffness and damping testing',
    license='BSD 3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_interface = blueberry_dc.arduino_interface:main',
            'data_subscriber = blueberry_dc.data_subscriber:main',
            'bag_reader = blueberry_dc.bag_reader:main',
            'ros_bag_to_csv = blueberry_dc.ros_bag_to_csv:main',
            'ros_bag_to_csv_old = blueberry_dc.ros_bag_to_csv_old:main',
            'ros_bag_play = blueberry_dc.ros_bag_play:main',
        ],
    },
)
