from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'auto_drive_network_bridge',
            executable = 'mock_server_node',
            name = 'mock_server',
            output = 'screen'
        ),
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='auto_drive_network_bridge',
                    executable='network_bridge_node',
                    name='network_bridge'
                )
            ]
        )
    ])