from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_params = PathJoinSubstitution(
        [FindPackageShare("vio_stream_bridge"), "config", "vio_stream_bridge.yaml"]
    )
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="ROS 2 parameter file for the VIO stream bridge",
            ),
            Node(
                package="vio_stream_bridge",
                executable="vio_stream_bridge",
                name="vio_stream_bridge",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
