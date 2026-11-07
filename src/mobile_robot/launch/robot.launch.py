from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mobile_robot',
            executable='fsm_node',
            name='fsm_node',
            output='screen'
        ),
        Node(
            package='mobile_robot',
            executable='robot_controller_node',
            name='robot_controller_node',
            output='screen'
        ),
        Node(
            package='mobile_robot',
            executable='camera_node',
            name='camera_node',
            output='screen'
        ),
        Node(
            package='mobile_robot',
            executable='detector_node',
            name='detector_node',
            output='screen'
        ),
        Node(
            package='mobile_robot',
            executable='status_node',
            name='status_node',
            output='screen'
        ),
        Node(
            package='mobile_robot',
            executable='web_node',
            name='web_node',
            output='screen'
        ),
        Node(
            package='mobile_robot',
            executable='serial_node',
            name='serial_node',
            output='screen',
            parameters=[{'simulation': False}]
        ),
    ])
