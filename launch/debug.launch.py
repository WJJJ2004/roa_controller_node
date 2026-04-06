from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = "roa_controller_node"
    params = os.path.join(get_package_share_directory(pkg), "config", "params.yaml")

    return LaunchDescription([

        Node(
            package=pkg,
            executable="roa_controller_node",
            name="roa_controller_node",
            output="screen",
            parameters=[
                params,
                {"REALTIME_CONTROL_MODE": False},
            ],
            arguments=["--ros-args", "--log-level", "info"],
        ),
        # Node(
        #     package=pkg,
        #     executable="system_manager_node",
        #     name="system_manager_node",
        #     output="screen",
        #     parameters=[params],
        #     arguments=["--ros-args", "--log-level", "info"],
        # )
    ])