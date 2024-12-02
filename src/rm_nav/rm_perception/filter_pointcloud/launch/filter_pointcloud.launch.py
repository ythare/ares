import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('filter_pointcloud'), 'config', 'filter_pointcloud.yaml')

    filter_pointcloud_node = Node(
        package='filter_pointcloud',
        executable='filter_pointcloud_node',
        namespace='',
        output='screen',
        parameters=[config,
                    {'use_sim_time': False}],
    )

    return LaunchDescription([filter_pointcloud_node])