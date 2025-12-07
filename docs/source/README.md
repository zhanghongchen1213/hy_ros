# HY ROS2 机器人项目

这是一个基于ROS2的综合机器人项目，集成了音频处理、计算机视觉、激光雷达和机器人建模等多个功能模块。

## 项目结构

```
hy_ws/
├── config/                    # 配置文件目录
│   ├── rviz_config.sh         # RViz配置脚本
│   ├── start.sh               # 项目启动脚本
│   └── start_terminal.sh      # 终端启动脚本
├── src/                       # 源代码目录
│   ├── all_launch/            # 统一启动包
│   ├── audio/                 # 音频处理包
│   ├── hy_urdf/              # 机器人URDF模型包
│   ├── img/                   # 图像处理相关包
│   │   ├── img_decode/        # 图像解码包
│   │   ├── object_track/      # 目标跟踪包
│   │   ├── rknn_yolov6/      # YOLO目标检测包
│   │   └── usb_camera/        # USB摄像头包
│   └── ldlidar_driver_ros2/   # 激光雷达驱动包
└── README.md                  # 项目说明文档
```

## 功能模块

### 1. 统一启动模块 (all_launch)
- **功能**: 提供统一的启动入口，协调各个功能模块的启动
- **类型**: C++ (ament_cmake)
- **依赖**: 所有其他功能包

### 2. 音频处理模块 (audio)
- **功能**: 音频数据的采集、处理和分析
- **类型**: Python (ament_python)
- **主要依赖**: rclpy

### 3. 机器人模型模块 (hy_urdf)
- **功能**: 机器人的URDF模型定义、可视化和仿真
- **类型**: Python (ament_python)
- **主要功能**:
  - 机器人模型描述
  - RViz可视化
  - Gazebo仿真支持
- **主要依赖**: 
  - robot_state_publisher
  - joint_state_publisher
  - xacro
  - rviz2
  - gazebo_ros

### 4. 图像处理模块群 (img/)

#### 4.1 USB摄像头模块 (usb_camera)
- **功能**: USB摄像头数据采集
- **类型**: C++ (ament_cmake)
- **主要依赖**: rclcpp, image_transport, sensor_msgs, cv_bridge

#### 4.2 图像解码模块 (img_decode)
- **功能**: 图像数据解码和预处理
- **类型**: C++ (ament_cmake)
- **主要依赖**: rclcpp, image_transport, sensor_msgs, cv_bridge

#### 4.3 YOLO目标检测模块 (rknn_yolov6)
- **功能**: 基于RKNN的YOLOv6目标检测
- **类型**: C++ (ament_cmake)
- **主要依赖**: rclcpp, vision_msgs, image_transport, cv_bridge

#### 4.4 目标跟踪模块 (object_track)
- **功能**: 目标跟踪和轨迹预测
- **类型**: C++ (ament_cmake)
- **主要依赖**: 
  - rclcpp, image_transport, cv_bridge
  - tf2, tf2_ros, tf2_geometry_msgs
  - vision_msgs, message_filters

### 5. 激光雷达模块 (ldlidar_driver_ros2)
- **功能**: LD06/STL-19P/LD19激光雷达驱动
- **类型**: C++ (ament_cmake)
- **主要依赖**: rclcpp, sensor_msgs
- **支持设备**: LD06, STL-19P, LD19

## 系统要求

- **操作系统**: Ubuntu 22.04 LTS
- **ROS版本**: ROS2 Humble
- **编译工具**: colcon
- **依赖库**: OpenCV, PCL等

## 安装和编译

### 1. 克隆项目
```bash
cd ~/
git clone <repository_url> hy_ws
cd hy_ws
```

### 2. 安装依赖
```bash
# 安装ROS2依赖
rosdep install --from-paths src --ignore-src -r -y

# 安装系统依赖
sudo apt update
sudo apt install -y python3-colcon-common-extensions
```

### 3. 编译项目
```bash
# 编译所有包
colcon build

# 或者单独编译某个包
colcon build --packages-select <package_name>
```

### 4. 设置环境
```bash
source install/setup.bash
```

## 使用方法

### 1. 启动完整系统
```bash
# 使用统一启动文件
ros2 launch all_launch all_launch.launch.py
```

### 2. 单独启动模块

#### 启动摄像头
```bash
ros2 launch usb_camera usb_camera_node.launch.py
```

#### 启动目标检测
```bash
ros2 launch rknn_yolov6 rknn_yolov6_node.launch.py
```

#### 启动激光雷达
```bash
ros2 launch ldlidar_driver_ros2 ldlidar_driver.launch.py
```

#### 启动机器人模型可视化
```bash
ros2 launch hy_urdf display.launch.py
```

### 3. 可视化

#### RViz可视化
```bash
rviz2
```
- 激光雷达数据: 选择"LaserScan"类型，话题选择"/scan"
- 机器人模型: 添加"RobotModel"显示
- 摄像头图像: 添加"Image"显示

## 配置说明

### 激光雷达配置
- 默认设备名: `/dev/ldlidar_serial`
- 设备别名配置: 运行 `ldlidar_udev.sh` 脚本
- 角度分割功能: 可通过参数 `enable_angle_crop_func` 控制

### NFS挂载配置
项目支持NFS网络文件系统，配置文件位于 `config/start.sh`:
- NFS服务器: 192.168.28.238
- 远程共享: /home/k/hy_linux/nfs
- 本地挂载点: /root/nfs

## 开发指南

### 添加新功能包
1. 在 `src/` 目录下创建新包
2. 更新 `all_launch` 包的依赖
3. 在统一启动文件中添加新包的启动配置

### 代码规范
- C++包使用 `ament_cmake` 构建系统
- Python包使用 `ament_python` 构建系统
- 遵循ROS2编码规范
- 使用Apache-2.0许可证

## 故障排除

### 常见问题
1. **编译错误**: 检查依赖是否完整安装
2. **设备权限问题**: 确保用户有设备访问权限
3. **网络连接问题**: 检查NFS服务器连接状态

### 调试工具
```bash
# 查看节点状态
ros2 node list

# 查看话题
ros2 topic list

# 查看话题数据
ros2 topic echo <topic_name>

# 查看节点信息
ros2 node info <node_name>
```

## 许可证

本项目采用Apache-2.0许可证，详见各包中的LICENSE文件。

## 贡献

欢迎提交Issue和Pull Request来改进项目。

## 联系方式

- 维护者: k
- 邮箱: johndoe@example.com

---

**注意**: 本项目在Ubuntu 20.04和ROS2 Humble环境下测试通过。使用前请确保系统环境配置正确。



# 香橙派3B环境

## 一、个人初始配置

1.1 修改root用户自动登录终端（密码orangepi）
```bash
sudo auto_login_cli.sh root
```
---
1.2 连接wifi(输入自己的wifi名和密码，并记录ip地址例如192.168.22.188)
```bash
sudo nmcli dev wifi connect wifi名称 password wifi密码
ip addr show wlan0
```
---
1.3 SSH远程连接测试（密码orangepi）
```bash
ssh root@192.168.22.188
```
---
## 二、挂载配置
### 2.1 挂载NFS
```bash
# 1. 安装和挂载NFS
sudo apt update
sudo apt install -y nfs-common
mkdir /mnt/nfs
# 手动挂载：注意事项：这里修改为自己电脑的 IP地址(192.168.22.177):/本地路径(/home/k/hy_linux/nfs) 香橙派挂载路径(/mnt/nfs)
sudo mount -t nfs -o nolock 192.168.22.177:/home/k/hy_linux/nfs /mnt/nfs

# 2. 可选: 配置nfs开机自动挂载
# 2.1 编辑~/.bash_profile，修改环境变量
# 打开~/.bash_profile文件
nano ~/.bash_profile
# 添加以下代码内容

if [ -f ~/.bashrc ]; then
    . ~/.bashrc
fi

# 2.2 编辑~/.bashrc，添加挂载命令
# 打开~/.bashrc文件
nano ~/.bashrc
# 添加以下代码内容
sudo mount -t nfs -o nolock 192.168.22.177:/home/k/hy_linux/nfs /mnt/nfs

```
---
### 2.2 挂载emmc(可选)
```bash
# 挂载emmc作为存储空间并配置开机自动挂载
# 查看所有存储设备和分区，查看是否有未挂载的eMMC分区
lsblk
# 格式化存储空间大的分区格式化为ext4（-F强制格式化，-L添加卷标hy_tmp，方便识别）(按实际情况修改mmcblk0p2)
sudo mkfs.ext4 -F -L hy_tmp /dev/mmcblk0p2
# 创建挂载点目录
sudo mkdir -p /hy_tmp
# 设置目录权限（允许所有用户读写执行，方便存放文件）
sudo chmod 777 /hy_tmp
# 设置目录所有者（可选，让当前用户拥有完整权限）
sudo chown $USER:$USER /hy_tmp
# 查看mmcblk0p2的UUID（复制输出中的UUID值，不含引号）
blkid /dev/mmcblk0p2
# 使用nano编辑器打开fstab（新手推荐nano，操作简单）
sudo nano /etc/fstab
# 添加以下内容到fstab文件末尾(将UUID替换为实际值)
UUID=实际UUID /hy_tmp ext4 defaults 0 2
# 查看是否挂载成功
df -h
# 读写测试
# 进入/hy_tmp目录
cd /hy_tmp
# 创建测试文件夹和文件
mkdir video audio screenshot
touch audio/test.mp3 video/test.mp4 screenshot/test.png
# 查看创建结果
ls -l
```

## 三、安装ros2
- 参考鱼香ros一键安装教程安装ros2
```bash
cd /opt
wget --no-check-certificate https://fishros.com/install -O fishros
./fishros
```
- 可在主机环境按照相同方法同步配置完ros2后，测试双端通信是否成功
  - 香橙派端侧：`ros2 run demo_nodes_cpp talker`
  - 主机侧：`ros2 run demo_nodes_cpp listener`

## 四、硬件测试
### 4.1 麦克风测试
- 测试命令：`arecord -l`
- 检查是否有麦克风设备（如card 2）
- 测试命令,修改2为本地card序号
```bash
arecord --device=plughw:2,0 --format S16_LE --rate 44100 -c 2 -d 5 /hy_tmp/record/test.wav
```
- 检查并下载test.wav文件是否录音成功

### 4.2 摄像头测试
```bash
# 配置推流环境
mkdir /opt/mjpg-streamer
cd /opt/mjpg-streamer
# 安装cmake和libjpeg8-dev（用于编译mjpg-streamer）
sudo apt-get install cmake libjpeg8-dev
# 通过ssh上传mjpg-streamer工具包
# 解压工具包
unzip master.zip
cd mjp*g-*
cd mjpg-*
make
sudo make install
cd /
# 安装v4l工具（用于查询摄像头参数）
sudo apt install -y v4l-utils
# 查看摄像头支持的格式、分辨率、帧率
v4l2-ctl --list-formats-ext
# 启动推流服务
/usr/local/bin/mjpg_streamer -i "/usr/local/lib/mjpg-streamer/input_uvc.so -d /dev/video0 -n -f 10 -r 1920x1080" -o "/usr/local/lib/mjpg-streamer/output_http.so -p 8085 -w /usr/local/share/mjpg-streamer/www"
```
- 开浏览器，输入香橙派IP地址加端口号（如：http://192.168.22.188:8085）即可查看实时视频流

### 4.3 雷达测试
```bash
# 新建应用程序空间
mkdir /opt/lidar
cd /opt/lidar
# 上传wheeltec_udev.sh、ldlidar_driver_python2.py和ldlidar_driver_python3.py文件到该目录
# 赋予执行权限
sudo chmod 777 wheeltec_udev.sh
sudo chmod 777 ld*
# 执行脚本
sudo ./wheeltec_udev.sh
# 重新插拔后检查更名是否成功（wheeltec_lidar -> ttyUSB0）
ll /dev  
# 检查python版本，安装pyserial库
python --version
sudo apt install -y python3-pip
sudo pip3 install pyserial
# 执行对应测试脚本（输出刷新说明雷达正常工作）
python3 ldlidar_driver_python3.py
```
### 4.4 串口启动
```bash
# 启动串口3
sudo orangepi-config
# 依次选择System->Hardware -> uart-m3 -> Space开启 -> save -> back -> Reboot
```

# 主机环境配置

## 个人初始配置

1. 确保与香橙派处于同一局域网络环境
2. 安装并启动NFS服务器服务，挂载本地NFS目录（可参考）
   - 服务器端（主机）
     - 安装服务：`sudo apt update && sudo apt install -y nfs-kernel-server`
     - 创建导出目录：`sudo mkdir -p /home/k/hy_linux/nfs && sudo chown -R k:k /home/k/hy_linux/nfs`
     - 编辑导出：`sudo bash -c "echo '/home/k/hy_linux/nfs 192.168.22.0/24(rw,sync,no_subtree_check,no_root_squash)' >> /etc/exports"`
     - 应用配置：`sudo exportfs -r && sudo systemctl enable --now nfs-kernel-server`
     - 查看导出：`sudo exportfs -v`
     - 防火墙放行（如启用 ufw）：`sudo ufw allow from 192.168.22.0/24 to any port nfs`
   - 注意事项
     - 必须启用 `no_root_squash`，否则客户端 root 无法写入，构建将出现权限错误
   - 示例网络：`/home/k/hy_linux/nfs 192.168.22.0/24(rw,sync,no_subtree_check,no_root_squash)` 已包含上述选项
  3. 一键安装鱼香ros2
4.
