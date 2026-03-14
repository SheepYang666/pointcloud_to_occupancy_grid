from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("pointcloud_to_occupancy_grid"), "config", "default.yaml"]
                ),
                description="Path to the ROS2 parameter file.",
            ),
            Node(
                package="pointcloud_to_occupancy_grid",
                executable="generate_pgm_from_keyframes",
                name="generate_pgm_from_keyframes",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
