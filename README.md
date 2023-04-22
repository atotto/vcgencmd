# VCGenCmd

This repository uses [vcgencmd](https://elinux.org/RPI_vcgencmd_usage) to add a ROS 2 node monitoring Raspberry Pi hardware. 

## Supported Features

* Temperature over sensor_msgs/msg/Temperature

## Usage


Build, test, and run

```commandline
colcon build
colcon test
. install/setup.bash
ros2 launch vcgencmd vcgen_monitor_launch.py
```

Verify output

```commandline
$ ros2 topic echo /pi_temperature 
header:
  stamp:
    sec: 1682182482
    nanosec: 651038735
  frame_id: pi
temperature: 37.9
variance: 0.0
---
```