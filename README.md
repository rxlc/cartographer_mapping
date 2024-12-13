# Cartographer Mapping with RPLidar

This project uses Google Cartographer to implement SLAM (lidar data only) with an RPLidar A1M8 sensor on ROS 2 Humble.

For more details on Cartographer configuration, refer to [Cartographer ROS documentation](https://google-cartographer-ros.readthedocs.io/) and [F1TENTH Autonomous Racing: Modern SLAM - Google Cartographer](https://www.youtube.com/watch?v=L51S2RVu-zc)

## Prerequisites

- ROS 2 Humble
- Rplidar ROS
- Cartographer ROS
- RViz2


## RPLidar Setup

Check USB permissions:
```
ls -l /dev/ttyUSB0
```

If necessary, set permissions for the RPLidar:
```
sudo chmod 666 /dev/ttyUSB0
```


## Mapping

Launch Cartographer SLAM:
```
ros2 launch cartographer_launch cartographer_rplidar.launch.py
```

RViz2 should automatically start and display the map being built.


## Saving the Map

To save the generated map:

In a new terminal, run:
```
ros2 run nav2_map_server map_saver_cli -f ./maps/my_map
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: './maps/my_map.pbstream'}"
```

This will save three files: `my_map.pgm` (the map image), `my_map.yaml` (the map metadata), and `my_map.pbstream` (proto stream)

## Pure Localization

To use pure localization mode:

Ensure you have saved your map files (.pbstream)

Then run:
```
ros2 launch cartographer_launch cartographer_localization.launch.py
```
