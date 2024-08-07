from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
import os

def generate_launch_description():
    # Get the current directory path (because the launch file is in the root directory)
    current_dir = os.path.dirname(os.path.abspath(__file__))

    # Path to the RViz configuration file (assuming it is in the rviz directory)
    rviz_config_path = os.path.join(current_dir,'..', 'rviz', 'auto_drive.rviz')

    return LaunchDescription([
        Node(
            package='sensor_simulator',
            executable='lidar_simulator',
            name='lidar_simulator',
            output='screen'
        ),
        
        Node(
            package='sensor_simulator',
            executable='vehicle_simulator',
            name='vehicle_simulator',
            output='screen'
        ),
        Node(
            package='perception',
            executable='obstacle_detection_node',
            name='obstacle_detection',
            output='screen'
        ),
        Node(
            package='perception',
            executable='obstacle_tracking_node',
            name='obstacle_tracking',
            output='screen'
        ),
        Node(
            package='planning',
            executable='path_planning_node',
            name='path_planning',
            output='screen'
        ),
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
        ),
        Node(
            package = 'control',
            executable = 'vehicle_control_node',
            name = 'vehicle_control',
            output = 'screen'
        ),
        Node(
            package='visualization',
            executable='auto_drive_visualizer_node',
            name='auto_drive_visualizer',
            output='screen'
        ),
        # Start RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    ])