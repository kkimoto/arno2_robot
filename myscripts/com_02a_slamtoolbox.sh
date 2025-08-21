
# prerequisite
# sudo apt install ros-humble-rmw-cyclonedds-cpp


#ros2 launch slam_toolbox online_sync_launch.py   slam_toolbox/scan:=/urg_node1/scan   slam_toolbox/odom:=/icart_mini/odom


ros2 launch slam_toolbox online_sync_launch.py   slam_toolbox/scan:=/merged_scan   slam_toolbox/odom:=/icart_mini/odom

