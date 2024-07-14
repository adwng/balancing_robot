from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the parameter file
    param_file = os.path.join(get_package_share_directory("bringup"), 'config', 'robot_odom.yaml')

    # Define the node
    odom_node = Node(
        package="robot_localization",
        executable="ekf_node",
        output="screen",
        parameters=[param_file],
        remappings=[('/odometry/filtered', '/odom')]
    )

    # Return the LaunchDescription
    return LaunchDescription([
        odom_node
    ])
