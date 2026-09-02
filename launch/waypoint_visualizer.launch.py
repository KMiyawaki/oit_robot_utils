from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. コマンドラインから受け取る引数を宣言
    target_file_arg = DeclareLaunchArgument(
        'target_file',
        default_value='',
        description='CSVファイルの絶対パス'
    )

    # 2. 引数を参照する変数を定義
    target_file = LaunchConfiguration('target_file')

    # 3. ノードの設定
    waypoint_visualizer_node = Node(
        package='oit_robot_utils',
        executable='waypoint_visualizer.py',
        name='waypoint_visualizer',
        output='screen',
        parameters=[{
            'csv_path': target_file
        }]
    )

    return LaunchDescription([
        target_file_arg,
        waypoint_visualizer_node
    ])