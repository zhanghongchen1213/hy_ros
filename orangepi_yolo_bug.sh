#!/bin/bash

# 一键启动串口节点
colcon build --packages-select rk_camera

source install/setup.bash

ros2 launch rk_camera rk_camera.launch.py