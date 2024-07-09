import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    prefix = get_package_share_directory("bringup")

    rviz_path = os.path.join(
        prefix, "rviz", "rviz.rviz"
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2', arguments=['-d', rviz_path],
        output='screen'
    )

    return LaunchDescription([
        rviz_node,
    ])