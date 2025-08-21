source ./install/setup.bash

(ros2 run tf2_ros static_transform_publisher --x 0.0 --y 0.0 --z 0.0 --roll 0.0 --pitch 0.0 --yaw 0.0 --frame-id base_link --child-frame-id laser1)&
echo laser1 tranlator started

# Kimoto experimental sensors.
#(ros2 run tf2_ros static_transform_publisher   --x 0.0 --y 0.25 --z 0.0   --roll 0.0 --pitch 0.0 --yaw 3.1415926535 --frame-id base_link   --child-frame-id laser2)&

# Arno sensors.
(ros2 run tf2_ros static_transform_publisher   --x 0.0 --y 0.25 --z 0.0   --roll 0.0 --pitch 0.0 --yaw -2.35619449 --frame-id base_link   --child-frame-id laser2)&
echo laser2 tranlator started

# Arno rosbag publishes odom to base_footprint.
# Additional base_footprint to base_link is necessary.
(ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0.01 --roll 0 --pitch 0 --yaw 0 --frame-id base_footprint --child-frame-id base_link)&
echo base_link to base_footprint tranlator started

ros2 launch laser_scan_merger start.launch.py robotname:=kimoto_ust2

