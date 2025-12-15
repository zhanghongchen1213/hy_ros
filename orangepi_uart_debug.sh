#!/bin/bash

# 一键启动串口节点
colcon build --packages-select pid_debug_interfaces uart

source install/setup.bash

ros2 launch uart uart.launch.py