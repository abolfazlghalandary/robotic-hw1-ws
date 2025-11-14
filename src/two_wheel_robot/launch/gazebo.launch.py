import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = FindPackageShare('two_wheel_robot').find('two_wheel_robot')
    
    xacro_file = os.path.join(pkg_share, 'urdf', 'two_wheel_robot.xacro')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'gazebo.rviz')
    world_file = os.path.join(pkg_share, 'worlds', 'empty.world')
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file]),
            'use_sim_time': True
        }],
        output='screen'
    )
    
    gazebo_server = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', 
             '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen'
    )
    
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )
    
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'two_wheel_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    delayed_spawn = TimerAction(
        period=3.0,
        actions=[spawn_robot]
    )
    
    lidar_transformer_node = Node(
        package='two_wheel_robot',
        executable='lidar_frame_transformer.py',
        name='lidar_frame_transformer',
        output='screen'
    )
    
    return LaunchDescription([
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        delayed_spawn,
        lidar_transformer_node,
        rviz
    ])
