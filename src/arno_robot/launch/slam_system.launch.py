from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    robotname = LaunchConfiguration("robotname")

    laser_scan_merger_launch = os.path.join(
        FindPackageShare("laser_scan_merger").find("laser_scan_merger"),
        "launch", "start.launch.py"
    )

    slam_params = os.path.join(
        FindPackageShare("slam_toolbox").find("slam_toolbox"),
        "config", "mapper_params_online_sync.yaml"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "robotname", default_value="kimoto_ust2",
            description="Robot name for laser_scan_merger"
        ),

        # laser_scan_merger
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(laser_scan_merger_launch),
            launch_arguments={
                "robotname": robotname
            }.items()
        ),

        # slam_toolbox (sync mode)
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params,
                {"use_sim_time": True}
            ],
            remappings=[
                ('/scan', '/merged_scan'),
                ('/odom', '/icart_mini/odom')
            ]
        )
    ])
