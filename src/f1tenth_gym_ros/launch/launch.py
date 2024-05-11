from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
import os


def generate_launch_description():
    ld = LaunchDescription()

    launches = [
        ("f1tenth_gym_ros", "/launch/gym_bridge_launch.py"),
        ("foxglove_bridge", "/foxglove_bridge_launch.xml"),
    ]
    # launch f1tenth_gym_ros simulation and foxglove studio bridge

    for package, launch_file in launches:
        if launch_file.endswith(".py"):
            ld.add_action(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [FindPackageShare(package).find(package) + launch_file]
                    )
                )
            )
        elif launch_file.endswith(".xml"):
            xml_launch_file_path = os.path.join(
                FindPackageShare(package).find(package), launch_file.strip("/")
            )
            ld.add_action(
                ExecuteProcess(
                    cmd=["ros2", "launch", xml_launch_file_path], output="screen"
                )
            )

    return ld
