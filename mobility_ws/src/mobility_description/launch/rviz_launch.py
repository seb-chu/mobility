import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config_path = os.path.join(get_package_share_directory('mobility_description'), 'rviz', 'rover.rviz')

    return LaunchDescription([
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=['-d', rviz_config_path],
            output="screen"
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{'robot_description': os.path.join(get_package_share_directory('mobility_description'), 'urdf', 'rover.urdf.xacro')}]
        ),
    ])
