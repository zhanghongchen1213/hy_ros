# 硬件环境配置

## 一、个人初始配置

1.1 修改 root 用户自动登录终端（密码 orangepi）

```shell
sudo auto_login_cli.sh root
```

---

1.2 连接 wifi(输入自己的 wifi 名和密码，并记录 ip 地址例如 192.168.22.188)

```shell
sudo nmcli dev wifi connect wifi名称 password wifi密码
sudo nmcli dev wifi connect ZHC_Web password 580231580231
ip addr show wlan0
```

---

1.3 SSH 远程连接测试（密码 orangepi）

```shell
ssh root@香橙派IP地址
```

---

## 二、挂载配置

### 2.1 挂载 NFS

```shell
# 1. 安装和挂载NFS
sudo apt update
sudo apt install -y nfs-common
mkdir /mnt/nfs
# 手动挂载，重启后不会自动挂载
# 注意事项：这里修改为自己电脑的 IP地址(192.168.22.177):/本地路径(/home/k/hy_linux/nfs) 香橙派挂载路径(/mnt/nfs)
sudo mount -t nfs -o nolock 192.168.22.177:/home/k/hy_linux/nfs /mnt/nfs

# 2. 配置nfs开机自动挂载
# 2.1 修改环境变量，设置开机自动加载~/.bashrc
echo -e 'if [ -f ~/.bashrc ]; then\n    . ~/.bashrc\nfi' >> ~/.bash_profile

# 2.2 修改环境变量，添加自动挂载nfs命令
echo 'sudo mount -t nfs -o nolock 192.168.22.177:/home/k/hy_linux/nfs /mnt/nfs' >> ~/.bashrc
source ~/.bash_profile
```

---

### 2.2 挂载 emmc(可选)

```shell
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

## 三、安装 ros2

- 参考鱼香 ros 一键安装教程安装 ros2

```shell
cd /opt
wget --no-check-certificate https://fishros.com/install -O fishros
chmod +x fishros
./fishros
# 1:一键安装 -> 1:更换系统源再继续安装 -> 2:更新系统源并清理第三方源 -> 1：自动测速选择最快的源 -> \
# 1:中科大镜像源 -> 1:humble(ROS2) -> 1:humble(ROS2)桌面版 -> 开始自动安装等待20min左右
```

```{figure} _static/5134F256-24EA-4568-9E1A-90A9392EEEA8.png
:alt: ROS2 安装界面
:width: 100%
:align: center
```

- 可在主机环境按照相同方法同步配置完 ros2 后，测试双端通信是否成功
  - 香橙派端侧：`ros2 run demo_nodes_cpp talker`
  - 主机侧：`ros2 run demo_nodes_cpp listener`

```{figure} _static/2D377DF7-6518-45F7-B95B-6FD6E9846352.png
:alt: 通信测试
:width: 100%
:align: center
```

## 四、硬件测试

### 4.1 摄像头测试

```shell
# 配置推流环境
mkdir /opt/camera
cd /opt/camera
# 安装cmake和libjpeg8-dev（用于编译mjpg-streamer）
sudo apt-get install cmake libjpeg8-dev
# 复制mjpg-streamer工具包到/opt/camera/目录下
cp /mnt/nfs/hy_ros/source/2.Camera/master.zip /opt/camera/
# 解压工具包
unzip master.zip
rm master.zip
cd mjp*g-*
make
sudo make install
# 安装v4l工具（用于查询摄像头参数）
sudo apt install -y v4l-utils
# 查看摄像头支持的格式、分辨率、帧率
v4l2-ctl --list-formats-ext
# 启动推流服务
/usr/local/bin/mjpg_streamer -i "/usr/local/lib/mjpg-streamer/input_uvc.so -d /dev/video0 -n -f 10 -r 1920x1080" -o "/usr/local/lib/mjpg-streamer/output_http.so -p 8085 -w /usr/local/share/mjpg-streamer/www"
```

- 开浏览器，输入香橙派 IP 地址加端口号（如：http://192.168.22.188:8085）即可查看实时视频流

```{figure} _static/3845F5C1-5EF4-48AD-A44E-9723432B73D7.png
:alt: 摄像头推流
:width: 100%
:align: center
```

### 4.2 雷达测试

```shell
# 新建应用程序空间
mkdir /opt/lidar
cd /opt/lidar
# 上传wheeltec_udev.sh脚本到该目录，上传ldlidar_driver_python3.py文件到该目录
cp /mnt/nfs/hy_ros/source/1.Lidar/wheeltec_udev.sh /opt/lidar/
cp /mnt/nfs/hy_ros/source/1.Lidar/ldlidar_driver_python3.py /opt/lidar/
# 赋予执行权限
sudo chmod 777 wheeltec_udev.sh
sudo chmod 777 ld*
# 执行脚本
sudo ./wheeltec_udev.sh
# 重新插拔后检查更名是否成功（wheeltec_lidar -> ttyUSB0）
ll /dev
# 检查python版本，安装pyserial库
sudo apt install -y python3-pip
sudo pip3 install pyserial
# 执行对应测试脚本（输出刷新说明雷达正常工作）
python3 ldlidar_driver_python3.py
```

```{figure} _static/C245E2A1-C5E6-403D-8423-C497E65EF1A5.png
:alt: 雷达串口配置
:width: 100%
:align: center
```

---

```{figure} _static/2954D3EC-4FC1-4569-A884-389B75EA8267.png
:alt: 雷达输出
:width: 100%
:align: center
```

### 4.3 麦克风测试

```shell
# 测试麦克风是否正常工作
arecord -l
# 测试麦克风录音
mkdir -p /tmp/record
arecord --device=plughw:3,0 --format S16_LE --rate 44100 -c 2 -d 5 /tmp/record/test.wav
# 检查并下载test.wav文件是否录音成功
```

### 4.4 串口启动

```shell
# 启动串口3
sudo orangepi-config
# 依次选择System->Hardware -> uart3-m2 -> Space开启 -> save -> back -> Reboot
```

# 应用环境配置

## 一、语音模型配置

### 1.1 Sherpa-ONNX 安裝

```shell
# 参考https://k2-fsa.github.io/sherpa/onnx/rknn/install.html#
# 1.安装sherpa-onnx，
mkdir /opt/sherpa-onnx
cd /opt/sherpa-onnx

# 1> 方式1：在线下载安装
pip install sherpa-onnx -f https://k2-fsa.github.io/sherpa/onnx/rk-npu-cn.html

# 2> 方式2：直接复制文件到/opt/sherpa-onnx/目录下
cp /mnt/nfs/hy_ros/source/0.RK3588S/1.sherpa-onnx/sherpa-onnx-cp310/sherpa_onnx-1.12.19-cp310-cp310-manylinux_2_27_aarch64.whl /opt/sherpa-onnx/
chmod +x /opt/sherpa-onnx/sherpa_onnx-1.12.19-cp310-cp310-manylinux_2_27_aarch64.whl
# 安装sherpa-onnx
pip3 install sherpa_onnx-1.12.19-cp310-cp310-manylinux_2_27_aarch64.whl
which sherpa-onnx

# 2.复制librknnrt.so文件到/opt/sherpa-onnx/目录下
cp /mnt/nfs/hy_ros/source/0.RK3588S/1.sherpa-onnx/sherpa-onnx-rknn-stream/librknnrt.so /opt/sherpa-onnx/
chmod +x /opt/sherpa-onnx/librknnrt.so
# 更新/lib/librknnrt.so 或 /usr/lib/librknnrt.so
cp /opt/sherpa-onnx/librknnrt.so /lib/
cp /opt/sherpa-onnx/librknnrt.so /usr/lib/

# 系统级修复libonnxruntime.so库动态链接问题
echo "$(python3 -c "import sherpa_onnx, os; print(os.path.join(os.path.dirname(sherpa_onnx.__file__), 'lib'))")" | sudo tee /etc/ld.so.conf.d/sherpa_onnx.conf
sudo ldconfig


# 3. 检查是否开启rknn支持，具体检查是否具有librknnrt.so库，若没有则参考连接手动安装
ldd $(which sherpa-onnx)
# 查看当前rknnrt版本，实测2.3.0版本无报错
strings /lib/librknnrt.so | grep "librknnrt version"
```

```{figure} _static/1EC69803-CE14-4303-8F42-F81402FB94D3.png
:alt: rknnrt 版本检查
:width: 100%
:align: center
```

---

### 1.2 部署流式 ASR 模型

```shell
# 参考https://k2-fsa.github.io/sherpa/onnx/rknn/models.html#
# 1> 方式1：在线下载文件
wget https://github.com/k2-fsa/sherpa-onnx/releases/download/asr-models/sherpa-onnx-rk3588-streaming-zipformer-small-bilingual-zh-en-2023-02-16.tar.bz2

# 2> 方式2：直接复制文件到/opt/sherpa-onnx/目录下
cp /mnt/nfs/hy_ros/source/0.RK3588S/1.sherpa-onnx/sherpa-onnx-rknn-stream/sherpa-onnx-rk3588-streaming-zipformer-small-bilingual-zh-en-2023-02-16.tar.bz2 /opt/sherpa-onnx/

# 2. 解压文件
tar xvf sherpa-onnx-rk3588-streaming-zipformer-small-bilingual-zh-en-2023-02-16.tar.bz2
rm sherpa-onnx-rk3588-streaming-zipformer-small-bilingual-zh-en-2023-02-16.tar.bz2

# 3. 可监控rknpu加载状态
watch -n 0.5 cat /sys/kernel/debug/rknpu/load

# 4. 解码文件,查看是否正常工作
sherpa-onnx \
  --provider=rknn \
  --encoder=./sherpa-onnx-rk3588-streaming-zipformer-small-bilingual-zh-en-2023-02-16/encoder.rknn \
  --decoder=./sherpa-onnx-rk3588-streaming-zipformer-small-bilingual-zh-en-2023-02-16/decoder.rknn \
  --joiner=./sherpa-onnx-rk3588-streaming-zipformer-small-bilingual-zh-en-2023-02-16/joiner.rknn \
  --tokens=./sherpa-onnx-rk3588-streaming-zipformer-small-bilingual-zh-en-2023-02-16/tokens.txt \
  ./sherpa-onnx-rk3588-streaming-zipformer-small-bilingual-zh-en-2023-02-16/test_wavs/4.wav


# 5. 实时音频测试
sherpa-onnx-alsa \
  --provider=rknn \
  --encoder=./sherpa-onnx-rk3588-streaming-zipformer-small-bilingual-zh-en-2023-02-16/encoder.rknn \
  --decoder=./sherpa-onnx-rk3588-streaming-zipformer-small-bilingual-zh-en-2023-02-16/decoder.rknn \
  --joiner=./sherpa-onnx-rk3588-streaming-zipformer-small-bilingual-zh-en-2023-02-16/joiner.rknn \
  --tokens=./sherpa-onnx-rk3588-streaming-zipformer-small-bilingual-zh-en-2023-02-16/tokens.txt \
  plughw:3,0
```

```{figure} _static/BBCDEE94-3D50-4D23-892F-11A2EEAB23D4.png
:alt: 模型解码
:width: 100%
:align: center
```

---

```{figure} _static/083E0A20-7679-4AFB-8526-9839DC6F158D.png
:alt: 实时音频
:width: 100%
:align: center
```

---

### 1.3 配置 RKLLM Qwen2-VL 模型

```shell
mkdir /opt/rknn-llm
cd /opt/rknn-llm
# 上传rknn-llm-release-v1.2.1.zip到/opt/rknn-llm/目录下
cp /mnt/nfs/hy_ros/source/0.RK3588S/2.rknn-llm/rknn-llm-release-v1.2.1b1.zip /opt/rknn-llm/

unzip rknn-llm-release-v1.2.1b1.zip
rm rknn-llm-release-v1.2.1b1.zip

# 进入Qwen2-VL_Demo/deploy目录
cd /opt/rknn-llm/rknn-llm-release-v1.2.1b1/examples/Qwen2-VL-2B_Demo/deploy
chmod +x build-linux.sh
./build-linux.sh

# 进入示例目录
cd install/demo_Linux_aarch64

# 复制模型文件到示例目录
cp /mnt/nfs/hy_ros/source/0.RK3588S/2.rknn-llm/Qwen2-VL-2B_llm_w8a8_rk3588.rkllm /opt/rknn-llm/
cp /mnt/nfs/hy_ros/source/0.RK3588S/2.rknn-llm/Qwen2-VL-2B_vision_rk3588.rknn /opt/rknn-llm/
chmod +x /opt/rknn-llm/Qw*

# 设置环境变量
export LD_LIBRARY_PATH=./lib

# 运行示例
./demo demo.jpg /opt/rknn-llm/Qwen2-VL-2B_vision_rk3588.rknn /opt/rknn-llm/Qwen2-VL-2B_llm_w8a8_rk3588.rkllm 128 512 3
```

```{figure} _static/EDB7D880-5FEF-4FD0-92C5-3C97A1CB05A9.png
:alt: Qwen2-VL 运行
:width: 100%
:align: center
```

---

```{figure} _static/71CD73B6-4883-4198-B71D-75B41F91C823.png
:alt: Qwen2-VL 结果
:width: 100%
:align: center
```

### 1.4 配置大模型声音复刻

```shell
# 参考链接https://cloud.baidu.com/doc/SPEECH/s/Am8wytft2

```

## 二、音频流配置

```shell
# 2.1 sherpa-onnx ros2_api需求库
sudo apt-get install libportaudio2  # 安装 PortAudio 库
pip3 install sounddevice # 安装 Python 音频处理库

# 2.2 rkllm ros2_api需求库
sudo apt-get install libopencv-dev python3-opencv # 安装 OpenCV 库
```

- 首先分析图片中的报错信息，然后中文输出报错信息，其次提供你认为报错的原因，最后给出你的解决方案并调整代码。不需要你直接在终端测试，因为你现在的环境是本机环境，与端侧环境无关，你只需要在本机环境下执行 colcon build 无报错即可，我会在端侧自行验证并给你提供反馈，咱们一起协同快速开发。
