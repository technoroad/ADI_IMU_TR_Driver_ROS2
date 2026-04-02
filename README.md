# ADI_IMU_TR_Driver_ROS2

ROS2 driver for Analog Devices MEMS IMU sensors connected via USB serial.

[Click here](https://github.com/technoroad/ADI_IMU_TR_Driver_ROS1) for ROS1 version.

## Demo movie

Click the thumbnail to open the youtube video.

[![Adi-IMU-TR](http://img.youtube.com/vi/2emmX7TSa1U/0.jpg)](https://www.youtube.com/watch?v=2emmX7TSa1U "Adi-IMU-TR")

## Supported Platforms

This driver supports two product lines, each with two generations:

|  | IMU Board | IMU Platform |
|--|-----------|-------------|
| **1st gen** | TR-IMU1647X (ADIS164xx) | TR-IMU-Platform |
| **2nd gen** | TR-IMU166XX (ADIS166xx) | TR-IMU-Platform2 |

Documentation is organized by generation and communication protocol:
- **1st gen** — CSV over Serial → [doc/gen1_csv.md](doc/gen1_csv.md)
- **2nd gen** — Binary over Serial → [doc/gen2_binary.md](doc/gen2_binary.md)

### How to identify your board

- IMU Board product number starts with **164xx** → 1st gen / **166xx** → 2nd gen
- IMU Platform board name: **TR-IMU-Platform** → 1st gen / **TR-IMU-Platform2** → 2nd gen

## Quick Start

### Install

```
$ cd [your package directory]
$ git clone --recursive https://github.com/technoroad/ADI_IMU_TR_Driver_ROS2
$ cd [your workspace directory]
$ rosdep update
$ rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y
```

### Build

```
$ cd [your workspace directory]
$ colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select adi_imu_tr_driver_ros2
$ source ./install/setup.bash
```

### Run

1st gen (CSV):
```
$ ros2 launch adi_imu_tr_driver_ros2 adis_rcv_csv.launch.py
```

2nd gen (Binary):
```
$ ros2 launch adi_imu_tr_driver_ros2 adis_rcv_bin.launch.py
```

See each platform's document for details.

## License

MIT
