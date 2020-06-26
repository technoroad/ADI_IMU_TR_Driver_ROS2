#The MIT License (MIT)
#Copyright (c) 2019 Techno Road Inc.
#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:
#The above copyright notice and this permission notice shall be included in
#all copies or substantial portions of the Software.
#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#THE SOFTWARE.

import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    imu = launch_ros.actions.Node(
        package='adi_imu_tr_driver_ros2',
        node_executable='adis_rcv_csv_node',
        output='screen',
        parameters=[{'__log_level': 'INFO',
                     'device': '/dev/ttyACM0',
                     'parent_id': 'odom',
                     'frame_id': 'imu',
                     'rate': 100.0,
                     }])

    rviz_config_dir = os.path.join(get_package_share_directory('adi_imu_tr_driver_ros2'), 'rviz', 'imu.rviz')
    rviz = Node(
        package='rviz2',
        node_executable='rviz2',
        node_name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': False}],
        output='screen')

    urdf = os.path.join(
        get_package_share_directory('adi_imu_tr_driver_ros2'),
        'urdf',
        'adis16470_breakout.urdf')

    rsp = Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        node_name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[urdf])
            
    return launch.LaunchDescription([
      imu, rviz, rsp
    ])
