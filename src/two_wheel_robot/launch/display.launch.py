from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, PathJoinSubstitution

def generate_launch_description():
    xacro_file = PathJoinSubstitution([
        FindPackageShare('two_wheel_robot'),
        'urdf',
        'two_wheel_robot.xacro'
    ])

    robot_description_content = Command(['xacro ', xacro_file])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('two_wheel_robot'),
        'rviz',
        'display.rviz'
    ])

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}],
        output='screen'
    )

    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz
    ])
