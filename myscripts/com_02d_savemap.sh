
ros2 run nav2_map_server map_saver_cli -f mymap

# NG #
#ros2 service call /slam_toolbox/save_map nav2_msgs/srv/SaveMap "{map_topic: 'map', map_url: 'my_map.yaml', image_format: 'pgm', map_mode: 'trinary', free_thresh: 0.25, occupied_thresh: 0.65}"

