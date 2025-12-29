#!/bin/bash

# 一键启动串口节点
colcon build --packages-select uart butter_robot_description all_launch pid_debug_interfaces

source install/setup.bash

ros2 launch all_launch all_launch.launch.py