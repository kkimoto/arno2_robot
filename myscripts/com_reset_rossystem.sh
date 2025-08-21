pkill -f static_transform_publisher
pkill -f ros2
sleep 3
ros2 daemon stop
sleep 3
ros2 daemon start
