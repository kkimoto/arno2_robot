import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, EmitEvent
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.events.lifecycle import ChangeState
from launch.events import matches_action
from ament_index_python.packages import get_package_share_directory
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    # Get params
    config_dir = os.path.join(get_package_share_directory('urg_node2'), 'config')
    with open(os.path.join(config_dir, 'params_ether.yaml'), 'r') as f:
        params_1st = yaml.safe_load(f)['urg_node2']['ros__parameters']
    with open(os.path.join(config_dir, 'params_ether_2nd.yaml'), 'r') as f:
        params_2nd = yaml.safe_load(f)['urg_node2']['ros__parameters']

    # urg_node2 lifecycle nodes
    node_1st = LifecycleNode(
        package='urg_node2',
        executable='urg_node2_node',
        name='urg_node2_1st',
        namespace='',
        remappings=[('scan', '/scan_1st')],
        parameters=[params_1st],
        output='screen'
    )
    node_2nd = LifecycleNode(
        package='urg_node2',
        executable='urg_node2_node',
        name='urg_node2_2nd',
        namespace='',
        remappings=[('scan', '/scan_2nd')],
        parameters=[params_2nd],
        output='screen'
    )

    # Static transforms
    tf_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser_1',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser_frame_1',
        ]
    )
    tf_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser_2',
        arguments=[
            '--x', '0.0',
            '--y', '0.25',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '3.1415926535',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser_frame_2',
        ]
    )

    # Composable laser scan merger
    laser_merger_container = ComposableNodeContainer(
        name='laser_scan_merger_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='laser_scan_merger',
                plugin='util::LaserScanMerger',
                name='laser_scan_merger_node',
                parameters=[os.path.join(get_package_share_directory('laser_scan_merger'), 'config', 'kimoto_ust', 'param.yaml')]
            )
        ],
        output='screen'
    )

    # Lifecycle transitions
    config_1 = TimerAction(
        period=2.0,
        actions=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(node_1st),
            transition_id=Transition.TRANSITION_CONFIGURE))]
    )
    activate_1 = TimerAction(
        period=3.0,
        actions=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(node_1st),
            transition_id=Transition.TRANSITION_ACTIVATE))]
    )
    config_2 = TimerAction(
        period=2.5,
        actions=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(node_2nd),
            transition_id=Transition.TRANSITION_CONFIGURE))]
    )
    activate_2 = TimerAction(
        period=3.5,
        actions=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(node_2nd),
            transition_id=Transition.TRANSITION_ACTIVATE))]
    )

    return LaunchDescription([
        node_1st,
        node_2nd,
        tf_1,
        tf_2,
        laser_merger_container,
        config_1,
        activate_1,
        config_2,
        activate_2
    ])
