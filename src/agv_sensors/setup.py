from setuptools import setup
from glob import glob
import os

package_name = 'agv_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),

    ('share/' + package_name, ['package.xml']),

    # cài toàn bộ file launch/*.py vào share/agv_sensors/launch
    (os.path.join('share', package_name, 'launch'),
     glob(os.path.join('launch', '*.py'))),

    # nếu có thư mục config (scan_filter.yaml, v.v.)
    (os.path.join('share', package_name, 'config'),
     glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Direct USB-RS485 wheel driver, lidar, and IMU nodes for AGV',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Node đọc IMU W901C-RS232
            'imu_node = agv_sensors.imu_node:main',

            # Node mới: /cmd_vel -> USB-RS485 Modbus -> ELD2-RS, publish /wheel_vel
            'wheel_vel_node = agv_sensors.wheel_vel_node_direct:main',
            'wheel_vel_node_nav = agv_sensors.wheel_vel_node_direct:main',
            'wheel_vel_node_direct = agv_sensors.wheel_vel_node_direct:main',

            'hwt901b_imu = agv_sensors.hwt901b_imu:main',
            'hwt901b_modbus = agv_sensors.khoa:main',
            
            'khoa = agv_sensors.khoa:main',
            
            'evaluation_straight10m = agv_sensors.evaluation_straight10m:main',
            # Nếu SAU NÀY em có file agv_sensors/lidar_node.py thì bật lại dòng này
            # 'lidar_node = agv_sensors.lidar_node:main',
            

        ],
    },
)
