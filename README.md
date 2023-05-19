# VCGenCmd

This repository uses [vcgencmd](https://elinux.org/RPI_vcgencmd_usage) to add a ROS 2 node monitoring Raspberry Pi hardware.

## Supported Features

* Temperature over sensor_msgs/msg/Temperature
* Throttling diagnostics over diagnostic_msgs/msg/DiagnosticArray

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

$ ros2 topic echo /diagnostics
---
header:
  stamp:
    sec: 1682192620
    nanosec: 729156030
  frame_id: pi
status:
- level: "\0"
  name: vcgen throttling
  message: vcgen healthy stats
  hardware_id: 10000000687a4fd8
  values:
  - key: Arm frequency capped
    value: Inactive
  - key: Soft temperature limit active
    value: Inactive
  - key: Under-voltage has occurred
    value: Inactive
  - key: Arm frequency capping has occurred
    value: Inactive
  - key: Throttling has occurred
    value: Inactive
  - key: Soft temperature limit has occurred
    value: Inactive
- level: "\x01"
  name: vcgen throttling
  message: vcgen unhealthy stats
  hardware_id: 10000000687a4fd8
  values:
  - key: Under-voltage detected
    value: Active
  - key: Currently throttled
    value: Active
---
```

## Python Vcgencmd

There is a [python implementation of vcgencmd](https://pypi.org/project/vcgencmd/). It is not yet used.
