
# before playing a bag slam_toolbox must have been launched.

#ros2 bag play ../bagdata/arno1/241124_atc/2024-11-24-11-07-02 --clock --remap /map:=/old_map --remap /map_metadata:=/old_map_metadata

ros2 bag play minimal_bag/minimal_bag_0.mcap --clock --rate 1.0


