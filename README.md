# adi_driver2 for ros2


## ADIS16xxx-TR

### Overview
“TR-IMU1647X” is an Analog Devices IMU sensor that can be easily connected to ROS and output high-precision attitude angles.

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
ROS2: dashing

### Connection

<span style="color: red; ">TODO</span>

### How to use
#### Build

``` $ roslaunch adi_driver adis_rcv_csv.launch ```

You can see the model of ADIS16470 breakout board in rviz panel.

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
