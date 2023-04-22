from launch import LaunchDescription
from launch_ros.actions import Node

PACKAGE = "vcgencmd"


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package=PACKAGE, executable="vcgen_monitor", name="vcgen_monitor"),
        ]
    )
