# ADI_IMU_TR_Driver_ROS2

### Overview
“TR-IMU1647X” is Analog Devices IMU sensor that can be easily connected to ROS and output high-precision attitude angles.

<div align="center">
  <img src="doc/TR-IMU16475-2.jpg" width="60%"/>
</div>

Currently supported devices are:

- [ADIS16470](https://www.analog.com/jp/products/adis16470.html)
  - Wide Dynamic Range Mini MEMS IMU

- [ADSI16475-2](https://www.analog.com/jp/products/adis16475.html)
  - Precision, Miniature MEMs IMU

Support for other sensors is possible by adding a library of sensors.

### Operating environment
OS： Ubuntu 18.04 LTS
ROS: ros2 eloquent

### Connection

<span style="color: red; ">TODO</span>

### How to use

#### Install
Go to your package directory and clone.
```
$ cd [your package directory]
$ git clone --recursive https://github.com/technoroad/ADI_IMU_TR_Driver_ROS2
```

Then resolve dependencies.
```
$ cd [your workspace directory]
$ rosdep update
$ rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y
```

#### Build
Go to your workspace directory and run the build command.  
```
$ cd [your workspace directory]
$ colcon build --symlink-install
```
Then set the path.
```
$ source ./install/setup.bash
```

#### Run
Execute with the following command.
```
$ ros2 launch adi_imu_tr_driver_ros2 adis_rcv_csv.launch.py
```

### Topics
- /imu/data_raw (sensor_msgs/Imu)

  IMU raw output. It contains angular velocities and linear
  accelerations. The orientation is always unit quaternion.  
  example:

```
$ ros2 tocpic echo /imu/data_raw
・・・
angular_velocity:
  x: -0.0116995596098
  y: -0.00314657808936
  z: 0.000579557116093
angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
linear_acceleration:
  x: 0.302349234658
  y: -0.303755252655
  z: 9.87837325989
linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
・・・
```

- /diagnostics (diagnostic_msgs/DiagnosticArray)

  Sensor state output.  
  example:

```
$ ros2 topic echo /diagnostics
・・・
header:
  seq: 80
  stamp:
    secs: 1587104853
    nsecs: 921894057
  frame_id: ''
status:
  -
    level: 0
    name: "adis_rcv_csv_node: imu"
    message: "OK"
    hardware_id: "ADIS16470"
    values: []
・・・
```
