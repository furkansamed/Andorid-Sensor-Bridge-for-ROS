from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_params = PathJoinSubstitution(
        [FindPackageShare("vio_stream_bridge"), "config", "vio_stream_bridge.yaml"]
    )
    default_cyclonedds = PathJoinSubstitution(
        [FindPackageShare("vio_stream_bridge"), "config", "cyclonedds.xml"]
    )
    params_file = LaunchConfiguration("params_file")
    cyclonedds_uri = LaunchConfiguration("cyclonedds_uri")
    diagnostics_enabled = LaunchConfiguration("diagnostics_enabled")
    diagnostics_period_sec = LaunchConfiguration("diagnostics_period_sec")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="ROS 2 parameter file for the VIO stream bridge",
            ),
            DeclareLaunchArgument(
                "cyclonedds_uri",
                default_value=default_cyclonedds,
                description="CycloneDDS XML profile used for ROS discovery",
            ),
            DeclareLaunchArgument(
                "diagnostics_enabled",
                default_value="false",
                description="Enable per-window publish diagnostics logging inside vio_stream_bridge",
            ),
            DeclareLaunchArgument(
                "diagnostics_period_sec",
                default_value="1.0",
                description="Publish diagnostics window length in seconds",
            ),
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
            SetEnvironmentVariable(
                "CYCLONEDDS_URI",
                [TextSubstitution(text="file://"), cyclonedds_uri],
            ),
            Node(
                package="vio_stream_bridge",
                executable="vio_stream_bridge",
                name="vio_stream_bridge",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "diagnostics_enabled": ParameterValue(
                            diagnostics_enabled,
                            value_type=bool,
                        ),
                        "diagnostics_period_sec": ParameterValue(
                            diagnostics_period_sec,
                            value_type=float,
                        ),
                    },
                ],
            ),
        ]
    )
