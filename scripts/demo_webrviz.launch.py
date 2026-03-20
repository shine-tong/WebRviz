from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    rosbridge_address = LaunchConfiguration("rosbridge_address")
    rosbridge_port = LaunchConfiguration("rosbridge_port")
    rosapi_params_timeout = LaunchConfiguration("rosapi_params_timeout")

    demo_launch = ExecuteProcess(
        cmd=[
            "/bin/bash",
            "-lc",
            (
                "set -o pipefail; "
                "stdbuf -oL -eL ros2 launch mr12_moveit_config demo.launch.py 2>&1 | "
                "stdbuf -oL -eL grep -E '^\\[(INFO|WARN|ERROR)\\] \\[launch\\]'"
            ),
        ],
        name="demo.launch.py",
        output="screen",
        emulate_tty=True,
    )

    rosbridge = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "address": rosbridge_address,
                "port": ParameterValue(rosbridge_port, value_type=int),
            }
        ],
    )

    rosapi = Node(
        package="rosapi",
        executable="rosapi_node",
        name="rosapi",
        output="log",
        parameters=[
            {
                "params_timeout": ParameterValue(
                    rosapi_params_timeout,
                    value_type=float,
                )
            }
        ],
    )

    observer = Node(
        package="webrviz_moveit_observer",
        executable="observer_node",
        name="webrviz_moveit_observer",
        output="log",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rosbridge_address",
                default_value="0.0.0.0",
                description="Bind address for rosbridge_websocket",
            ),
            DeclareLaunchArgument(
                "rosbridge_port",
                default_value="9090",
                description="Bind port for rosbridge_websocket",
            ),
            DeclareLaunchArgument(
                "rosapi_params_timeout",
                default_value="15.0",
                description="Timeout in seconds for rosapi parameter discovery",
            ),
            demo_launch,
            rosbridge,
            rosapi,
            observer,
        ]
    )
