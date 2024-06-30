from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取当前目录路径（因为 launch 文件在根目录）
    current_dir = os.path.dirname(os.path.abspath(__file__))

    # RViz配置文件的路径（假设它在 rviz 目录下）
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
        # 启动 RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    ])