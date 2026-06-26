from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='oit_robot_utils',
            executable='waypoint_visualizer.py',
            name='waypoint_visualizer',
            output='screen',
            parameters=[
                {
                    'csv_path': '/home/ubuntu/ros2_ws/src/oit_navigation/csv/points.csv'
                }
            ]
        )
    ])