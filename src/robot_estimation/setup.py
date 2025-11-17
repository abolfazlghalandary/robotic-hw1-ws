from setuptools import setup
import os
from glob import glob

package_name = 'robot_estimation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Robot estimation package with IMU filtering, motor control, and odometry',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_filter = robot_estimation.imu_filter:main',
            'motor_controller = robot_estimation.motor_controller:main',
            'odometry_estimator = robot_estimation.odometry_estimator:main',
        ],
    },
)
