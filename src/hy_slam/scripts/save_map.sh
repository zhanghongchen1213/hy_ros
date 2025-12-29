#!/bin/bash
# 功能：保存 slam_toolbox 构建的地图
# 用法：./save_map.sh [地图名]

MAP_NAME=${1:-my_map}
SAVE_DIR=$(ros2 pkg prefix hy_slam)/share/hy_slam/map

# 确保保存目录存在
mkdir -p $SAVE_DIR

# 调用 ROS2 服务保存地图
# slam_toolbox 提供了 /slam_toolbox/save_map 服务
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '$SAVE_DIR/$MAP_NAME'}}"

echo "Map saved to: $SAVE_DIR/$MAP_NAME"
