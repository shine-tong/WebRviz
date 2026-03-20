from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package="webrviz_moveit_observer",
            executable="observer_node",
            name="webrviz_moveit_observer",
            output="screen",
        )
    ])
