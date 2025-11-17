from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_estimation',
            executable='imu_filter',
            name='imu_filter',
            output='screen',
            parameters=[{
                'lowpass_alpha': 0.2,
                'complementary_alpha': 0.98
            }]
        ),
        
        Node(
            package='robot_estimation',
            executable='motor_controller',
            name='motor_controller',
            output='screen',
            parameters=[{
                'wheel_radius': 0.1,
                'wheel_base': 0.3
            }]
        ),
        
        Node(
            package='robot_estimation',
            executable='odometry_estimator',
            name='odometry_estimator',
            output='screen',
            parameters=[{
                'wheel_radius': 0.1,
                'wheel_base': 0.3
            }]
        ),
    ])
