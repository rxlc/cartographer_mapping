# Cartographer Mapping with RPLidar

This project uses Google Cartographer to implement SLAM (lidar data only) with an RPLidar A1M8 sensor on ROS 2 Humble.

For more details on Cartographer configuration, refer to [Cartographer ROS documentation](https://google-cartographer-ros.readthedocs.io/).

## Prerequisites

- ROS 2 Humble
- Cartographer ROS
- RViz2

## RPLidar Setup

1. Check USB permissions:
```
ls -l /dev/ttyUSB0
```


2.  If necessary, set permissions for the RPLidar:
```
sudo chmod 666 /dev/ttyUSB0
```


## Running SLAM

Launch Cartographer SLAM:
```
ros2 launch cartographer_launch cartographer_rplidar.launch.py
```

RViz2 should automatically start and display the map being built.


## Saving the Map

To save the generated map:

In a new terminal, run:
```
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

```

Replace `my_map` with your desired filename.

This will save two files: `my_map.pgm` (the map image) and `my_map.yaml` (the map metadata).


