import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    # Set the path to the SLAM Toolbox configuration
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    online_sync_launch = os.path.join(slam_toolbox_dir, 'launch', 'online_sync_launch.py')

    # Command to publish to /initial_pose
    publish_initial_pose_cmd = ExecuteProcess(
            cmd=[FindExecutable(name='ros2'), 'topic', 'pub', '/initialpose', 'geometry_msgs/msg/PoseWithCovarianceStamped', '\'{ "header": { "frame_id": "map" }, "pose": { "pose": { "position": { "x": 253.19850747840297, "y": 235.7077534304758, "z": 0 }, "orientation": { "x": 0, "y": 0, "z": 0.49574867128912475, "w": 0.8684660355564099 } } } }\'', '--once'],
        shell=True
    )
    
    # Include SLAM Toolbox launch file
    delay = 1.0  # Delay in seconds
    slam_toolbox_launch = TimerAction(
        period=delay,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([online_sync_launch]),
            )
        ]
    )

    return LaunchDescription([
        publish_initial_pose_cmd,
        slam_toolbox_launch
    ])

