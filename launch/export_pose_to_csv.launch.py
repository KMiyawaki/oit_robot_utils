
from launch_ros.actions import Node

from launch import LaunchDescription
from oit_robot_utils.launch_utils import PackagePath, declare_arg


def generate_launch_description():
    """
    Launch the pose_to_csv exporter node.
    """
    pkg_path = PackagePath('oit_robot_utils')

    # Declare a launch argument for the parameters file
    params_file_arg = declare_arg(
        'params_file',
        default_value=pkg_path.join('config', 'export_pose_to_csv.yaml'),
        description='Path to the parameters file for the pose exporter node.'
    )

    # Define the node
    export_pose_node = Node(
        package='oit_robot_utils',
        executable='export_pose_to_csv.py',
        name='export_pose_to_csv',
        output='screen',
        parameters=[params_file_arg.conf]
    )

    return LaunchDescription([
        params_file_arg.arg,
        export_pose_node,
    ])
