from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction
import os

def generate_launch_description():
    # Paths
    slam_toolbox_launch_file = os.path.join(
        FindPackageShare("slam_toolbox").find("slam_toolbox"),
        "launch", "online_sync_launch.py"
    )

    laser_scan_merger_launch_file = os.path.join(
        FindPackageShare("laser_scan_merger").find("laser_scan_merger"),
        "launch", "start.launch.py"
    )

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            "robotname", default_value="kimoto_ust2",
            description="Robot name for laser_scan_merger"
        ),

        # Static TF: base_link → laser1
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_base_link_to_laser1",
            arguments=[
                "--x", "0.0", "--y", "0.0", "--z", "0.0",
                "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                "--frame-id", "base_link", "--child-frame-id", "laser1"
            ]
        ),

        # Static TF: base_link → laser2
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_base_link_to_laser2",
            arguments=[
                "--x", "0.0", "--y", "0.25", "--z", "0.0",
                "--roll", "0.0", "--pitch", "0.0", "--yaw", "-2.35619449",
                "--frame-id", "base_link", "--child-frame-id", "laser2"
            ]
        ),

        # Static TF: base_footprint → base_link
#        Node(
#            package="tf2_ros",
#            executable="static_transform_publisher",
#            name="static_tf_base_footprint_to_base_link",
#            arguments=[
#                "--x", "0.0", "--y", "0.0", "--z", "0.01",
#                "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
#                "--frame-id", "base_footprint", "--child-frame-id", "base_link"
#            ]
#        ),
        TimerAction(
            period=1.0,  # Delay in seconds
            actions=[
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name="static_tf_base_link_to_laser2",
                    arguments=[
                        "--x", "0.0", "--y", "0.25", "--z", "0.0",
                        "--roll", "0.0", "--pitch", "0.0", "--yaw", "-2.35619449",
                        "--frame-id", "base_link", "--child-frame-id", "laser2"
                    ]
                )
            ]
        ),

        # SLAM Toolbox with remaps
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch_file),
            launch_arguments={
                "slam_toolbox/scan": "/merged_scan",
                "slam_toolbox/odom": "/icart_mini/odom"
            }.items()
        ),

        # Laser scan merger
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(laser_scan_merger_launch_file),
            launch_arguments={
                "robotname": LaunchConfiguration("robotname")
            }.items()
        ),
    ])
