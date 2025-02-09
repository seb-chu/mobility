import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_path = get_package_share_directory('mobility_description')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'gazebo.launch.py')
        )
    )

    return LaunchDescription([
        gazebo,
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=["-entity", "rover", "-file", os.path.join(pkg_path, "urdf", "rover.urdf.xacro")],
            output="screen"
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen"
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{'robot_description': os.path.join(pkg_path, 'urdf', 'rover.urdf.xacro')}]
        ),
    ])
