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
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition

def generate_launch_description():

    urdf = os.path.join(
        get_package_share_directory('adi_imu_tr_driver_ros2'),
        'urdf',
        'adis16470_breakout.urdf')
    
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
    
    rviz_config_dir = os.path.join(get_package_share_directory('adi_imu_tr_driver_ros2'), 'rviz', 'imu.rviz')

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            name="with_rviz",
            default_value="True",
            description="Launch Rviz2?"),
        DeclareLaunchArgument(
            name="device",
            default_value="/dev/ttyACM0",
            description="Device name"),
        DeclareLaunchArgument(
            name="mode",
            default_value="Attitude",
            description="Choice mode 'Attitued' or 'Register'"),
        DeclareLaunchArgument(
            name="parent_id",
            default_value="odom",
            description="Name of parent frame"),
        DeclareLaunchArgument(
            name="frame_id",
            default_value="imu",
            description="The frame name where the imu is mounted."),
        DeclareLaunchArgument(
            name="rate",
            default_value="100.0",
            description="Publish rate."),
        Node(
          package='adi_imu_tr_driver_ros2',
          executable='adis_rcv_csv_node',
          output='screen',
          parameters=[{'__log_level': 'INFO',
                       'device': LaunchConfiguration("device"),
                       'parent_id': LaunchConfiguration("parent_id"),
                       'frame_id': LaunchConfiguration("frame_id"),
                       'rate': LaunchConfiguration("rate"),
                       'mode': LaunchConfiguration("mode"),
                       }]),
        Node(
          package='robot_state_publisher',
          executable='robot_state_publisher',
          name='robot_state_publisher',
          output='log',
          parameters=[{'use_sim_time': False, 'robot_description': robot_desc}],
          arguments=[urdf]),
        Node(
          package='rviz2',
          executable='rviz2',
          name='rviz2',
          arguments=['-d', rviz_config_dir],
          parameters=[{'use_sim_time': False}],
          output='log',
          condition=IfCondition(LaunchConfiguration("with_rviz")))
    ])

