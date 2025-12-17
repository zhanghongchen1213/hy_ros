# 一、主机环境配置

本项目主机环境为 Ubuntu22.04 系统，确保与端侧处于同一局域网络环境

1. 启动本项目文件中配置 NFS 服务器脚本，挂载本地 NFS 目录: `sudo nfs_config.sh`
2. 将本项目文件放到`/home/${SUDO_USER:-$USER}/hy_linux/nfs`目录下
3. 参考教程：[鱼香 ROS2 一键安装](https://blog.csdn.net/weixin_71683006/article/details/150465016?ops_request_misc=elastic_search_misc&request_id=dd0d7d3fd479c65031b7b2767c112822&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~top_positive~default-1-150465016-null-null.142^v102^pc_search_result_base3&utm_term=%E9%B1%BC%E9%A6%99ros2%E4%B8%80%E9%94%AE%E5%AE%89%E8%A3%85&spm=1018.2226.3001.4187)

```shell
# 一键配置ROS2 humble开发环境
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

  - 端侧：`ros2 run demo_nodes_cpp talker`
  - 主机侧：`ros2 run demo_nodes_cpp listener`

```{figure} _static/2D377DF7-6518-45F7-B95B-6FD6E9846352.png
:alt: 通信测试
:width: 100%
:align: center
```

---

5. 安装必要的 python 库

```shell
# 用于语音复刻模型
sudo apt-get install python3-aiohttp python3-websockets
```

# 二、端侧环境配置

## 1. 个人初始配置

1.1 烧录镜像文件 **Orangepi5pro_1.0.6_ubuntu_jammy_server_linux5.10.160** 到 tf 卡中

---

1.2 修改 root 用户自动登录终端（密码 **orangepi**）

```shell
sudo auto_login_cli.sh root
```

---

1.2 连接 wifi

**注意**：输入自己的 wifi 名和密码，并记录 ip 地址

```shell
sudo nmcli dev wifi connect wifi名称 password wifi密码
ip addr show wlan0
```

---

1.3 SSH 远程连接测试（密码 orangepi）

**注意**：替换为端侧 ip 地址。例如 192.168.22.188

```shell
ssh root@端侧IP地址
```

---

## 2. 挂载配置

### 2.1 挂载 NFS

```shell
# 1. 安装和挂载NFS
sudo apt update
sudo apt install -y nfs-common
mkdir /mnt/nfs
# 手动挂载，重启后不会自动挂载
# 注意事项：这里修改为ubuntu主机侧IP地址(192.168.22.177):/nfs开发路径(/home/k/hy_linux/nfs) 端侧挂载路径(/mnt/nfs)
sudo mount -t nfs -o nolock 主机IP地址:/主机挂载路径 端侧挂载路径
sudo mount -t nfs -o nolock 192.168.22.177:/home/k/hy_linux/nfs /mnt/nfs

# 2. 配置nfs开机自动挂载
# 2.1 修改环境变量，设置开机自动加载~/.bashrc
echo -e 'if [ -f ~/.bashrc ]; then\n    . ~/.bashrc\nfi' >> ~/.bash_profile

# 2.2 修改环境变量，添加自动挂载nfs命令并激活环境
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

## 3. 安装 ROS2 humble

参考教程：[鱼香 ROS2 一键安装](https://blog.csdn.net/weixin_71683006/article/details/150465016?ops_request_misc=elastic_search_misc&request_id=dd0d7d3fd479c65031b7b2767c112822&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~top_positive~default-1-150465016-null-null.142^v102^pc_search_result_base3&utm_term=%E9%B1%BC%E9%A6%99ros2%E4%B8%80%E9%94%AE%E5%AE%89%E8%A3%85&spm=1018.2226.3001.4187)

```shell
# 一键配置ROS2 humble开发环境
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

  - 端侧：`ros2 run demo_nodes_cpp talker`
  - 主机侧：`ros2 run demo_nodes_cpp listener`

```{figure} _static/2D377DF7-6518-45F7-B95B-6FD6E9846352.png
:alt: 通信测试
:width: 100%
:align: center
```

## 4. 硬件测试

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

- 打开浏览器，输入**端侧 IP 地址加端口号**（如：http://192.168.22.188:8085）即可查看实时视频流

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
# 查看麦克风设备列表
arecord -l
# 测试麦克风录音
mkdir -p /tmp/record
arecord --device=plughw:3,0 --format S16_LE --rate 44100 -c 2 -d 5 /tmp/record/test.wav
# 参数说明：
# --device=plughw:3,0：指定使用的麦克风设备（根据arecord -l输出的设备列表修改）
# --format S16_LE：指定音频格式为16位小端字节序
# --rate 44100：指定采样率为44100Hz
# -c 2：指定通道数为2（立体声）
# -d 5：指定录音时长为5秒
# /tmp/record/test.wav：指定输出文件路径
# 检查并下载test.wav文件是否录音成功
```

### 4.4 串口启动

```shell
# 启动串口3
sudo orangepi-config
# 依次选择System->Hardware -> uart3-m2 -> Space开启 -> save -> back -> Reboot
```

# 三、语音模型调试

## 1. 部署 Sherpa-ONNX-RKNPU 流式 ASR 模型

- **参考：**[Sherpa-ONNX for RKNPU](https://k2-fsa.github.io/sherpa/onnx/rknn/install.html#)
- 安装 Sherpa-ONNX-RKNPU

```shell
# 1. ROS2功能包API调用库
sudo apt-get install libportaudio2  # 安装 PortAudio 库
pip3 install sounddevice # 安装 Python 音频处理库

# 2.安装sherpa-onnx，
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

# 3.复制librknnrt.so文件到/opt/sherpa-onnx/目录下
cp /mnt/nfs/hy_ros/source/0.RK3588S/1.sherpa-onnx/sherpa-onnx-rknn-stream/librknnrt.so /opt/sherpa-onnx/
chmod +x /opt/sherpa-onnx/librknnrt.so

# 更新/lib/librknnrt.so 或 /usr/lib/librknnrt.so
cp /opt/sherpa-onnx/librknnrt.so /lib/
cp /opt/sherpa-onnx/librknnrt.so /usr/lib/

# 4. 系统级修复libonnxruntime.so库动态链接问题，为后续ros2 C++开发提供支持
echo "$(python3 -c "import sherpa_onnx, os; print(os.path.join(os.path.dirname(sherpa_onnx.__file__), 'lib'))")" | sudo tee /etc/ld.so.conf.d/sherpa_onnx.conf
sudo ldconfig


# 5. 检查是否开启rknn支持，具体检查是否具有librknnrt.so库，若没有则参考连接手动安装
ldd $(which sherpa-onnx)
# 查看当前rknnrt版本，实测2.3.0版本可用无报错
strings /lib/librknnrt.so | grep "librknnrt version"
```

```{figure} _static/1EC69803-CE14-4303-8F42-F81402FB94D3.png
:alt: rknnrt 版本检查
:width: 100%
:align: center
```

---

- **参考：**[pre-trained models for RKNPU](https://k2-fsa.github.io/sherpa/onnx/rknn/models.html#)
- 下载预训练模型并终端测试 rknpu-sherpa-onnx 模型

```shell
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

## 2. 部署 RKLLM Qwen3-VL-2B 模型

- **参考：**[RKLLM 模型转换](https://doc.embedfire.com/linux/rk356x/Ai/zh/latest/lubancat_ai/example/qwen2_vl.html)
- **参考：**[Qwen-VL-2B 部署 RK3588S](https://blog.csdn.net/lajuchenghui/article/details/150111489)

```shell
# 1. 安装OpenCV库，Qwen3-VL-2B ROS功能包多模态输入需求库
sudo apt-get install libopencv-dev python3-opencv # 安装 OpenCV 库

# 2.新建目录
mkdir /opt/rknn-llm
cd /opt/rknn-llm
# 上传rknn-llm-release-v1.2.1.zip到/opt/rknn-llm/目录下
cp /mnt/nfs/hy_ros/source/0.RK3588S/2.rknn-llm/rknn-llm.zip /opt/rknn-llm/
unzip rknn-llm.zip
rm rknn-llm.zip

# 3.进入multimodal_model_demo/deploy目录
cd /opt/rknn-llm/rknn-llm/examples/multimodal_model_demo/deploy
chmod +x build-linux.sh
./build-linux.sh

# 4.更新librkllmrt.so库，为后续ros2 C++开发提供支持
cp /opt/rknn-llm/rknn-llm/rkllm-runtime/Linux/librkllm_api/aarch64/librkllmrt.so /lib/
chmod +x /lib/librkllmrt.so

# 5. 更新rkllm.h头文件，为后续ros2 C++开发提供支持
cp /opt/rknn-llm/rknn-llm/rkllm-runtime/Linux/librkllm_api/include/rkllm.h /usr/include/

# 6.复制rkllm模型文件
cd install/demo_Linux_aarch64

# 复制模型文件到示例目录
cp /mnt/nfs/hy_ros/source/0.RK3588S/2.rknn-llm/qwen3-vl-2b_vision_rk3588.rknn /opt/rknn-llm/
cp /mnt/nfs/hy_ros/source/0.RK3588S/2.rknn-llm/qwen3-vl-2b-instruct_w8a8_rk3588.rkllm /opt/rknn-llm/
chmod +x /opt/rknn-llm/qwen*

# 7.设置环境变量
export LD_LIBRARY_PATH=./lib

# 8. 运行示例
./demo demo.jpg /opt/rknn-llm/qwen3-vl-2b_vision_rk3588.rknn /opt/rknn-llm/qwen3-vl-2b-instruct_w8a8_rk3588.rkllm 2048 4096 3
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

## 3. 部署百度大模型音色复刻 TTS

- **参考：**[百度大模型音色复刻 TTS](https://cloud.baidu.com/doc/SPEECH/s/Am8wytft2)

1. 安装必要的 python 库

> **注意：** 主机侧和端侧均需要安装
>
> ```shell
> # 安装异步 http 库和 websockets 库
> sudo apt-get install python3-aiohttp python3-websockets
> ```

2. 录制 15s 任意文本音频, 并转换为 base64 编码的 mp3 格式

> - [在线转换工具](https://base64.guru/converter/encode/audio/mp3)
> - 下载 base64.txt 文件,替换`hy_ros/src/audio_tts/audio_tts/base64.txt`

3. 获取 API key

> - 参考[百度大模型应用创建](https://ai.baidu.com/ai-doc/REFERENCE/Bkru0l60m)创建应用配置百度智能云 api 密钥，Server Key
> - 更新到秘钥到`hy_ros/src/audio_tts/launch/audio_tts_launch.py` 文件中和`tts_api.py` 文件中

4. 创建 Voice ID

> **注意：** 可直接在主机侧运行创建并更新 Voice ID
>
> - 在 `hy_ros/src/audio_tts/audio_tts` 目录下运行`python3 tts_api.py`
> - 查看并记录返回的 Voice ID
> - 更新到 `hy_ros/src/audio_tts/launch/audio_tts_launch.py` 文件中

---

```{figure} _static/{EE70FF84-01DA-4F41-B397-768A63B5CD34}.png
:alt: 语音复刻模型
:width: 100%
:align: center
```

---

```{figure} _static/3AA2DD25-37C9-448F-9655-7A191EE35A87.png
:alt: 语音复刻模型测试
:width: 100%
:align: center
```

# 四、Foxglove Web 可视化配置

## 1. 端侧部署 (RK3588S)

Foxglove Bridge 用于将 ROS 2 话题通过 WebSocket 转发给 Foxglove Studio。

```bash
# 配置 joint_state_publisher,实现关节状态发布
sudo apt-get install -y ros-humble-joint-state-publisher
# 配置 foxglove_bridge,实现话题转发
sudo apt-get install -y ros-humble-foxglove-bridge
# 配置 RTSP 转 WebRTC 网关，使用 go2rtc拉取硬件编码 RTSP 流

#
```

## 2. 上位机连接 (PC/iPad)

1. 打开 [Foxglove Studio](https://foxglove.dev/) (网页版或客户端)。
2. 点击 **"Open Connection"**。
3. 选择 **"Foxglove WebSocket"** 协议。
4. 输入端侧地址：`ws://192.168.22.219:8765` (IP 根据实际情况调整)。
5. 连接成功后即可订阅和查看端侧的所有话题。

```{figure} _static/{2BFCE0F0-25D7-44CC-A168-7EC55C00E17A}.png
:alt: Foxglove 连接
:width: 100%
:align: center
```

# 五、 yolov8 目标检测模型训练

如果希望自己训练 yolov8 黄油或其他目标检测模型，以下教程可提供参考；

## 1. 数据集准备

1. 运行 `tools/capture_tool.py` 采集数据集, 采集到的图像会保存到 `source/butter_img` 目录下。（建议采集 100 张以上图像）
2. 使用 [https://app.roboflow.com](https://app.roboflow.com) 平台实现数据集标注和管理，

> ```{figure} _static/{5F43021B-F98A-4638-9F1C-1A0F37A3776A}.png
> :alt: 数据集标注
> :width: 100%
> :align: center
> ```

> ```{figure} _static/{9EBE7DA9-24AF-410B-ADC4-2D899B346B32}.png
> :alt: 数据集标注，划分
> :width: 100%
> :align: center
> ```

3. 数据集预处理包括数据增强、数据划分，下载 yolov8 数据集配置文件

> ```{figure} _static/{F60B123F-AA37-4341-8C82-F983E41C8873}.png
> :alt: 数据集预处理
> :width: 100%
> :align: center
> ```

## 2. Win 环境实现模型训练

### 2.1 配置虚拟环境

1. 初始化 uv 项目

```powershell
# 安装uv
pip3 install uv
# 查看python版本
uv python list
# 选择python版本创建虚拟环境项目
uv init -p 3.13.2 butter_train_env
cd butter_train_env
```

2. 添加依赖

```powershell
uv add pandas  # 安装pandas库
uv add --dev ipykernel  # 安装ipykernel库，用于在Jupyter中选择项目内核
```

3. 创建并注册项目内核

> 在项目虚拟环境下注册一个可在 Jupyter/Trae 中选择的内核：

> ```powershell
> .venv\Scripts\python -m ipykernel install --user --name butter_train > --display-name "Python (butter_train)"
> ```

> 完成后，系统目录 `C:\Users\<你的用户名> >\AppData\Roaming\jupyter\kernels\butter_train` 会出现对应的 `kernelspec`。

4. 在 VSCode 或者 Pycharm IDE 中选择内核

> - 打开 Notebook，使用右上角的内核选择器切换到：`Python (butter_train)`
> - 选择后，`import pandas as pd` 即可在该项目环境中正常使用。

5. UV 常用命令

> ```shell
> uv tree  # 查看项目依赖树
> uv sync  # 根据 `pyproject.toml` 同步安装依赖到 `.venv`
> uv sync --no-dev  # 仅安装非开发依赖
> uv add 'pandas==2.31.0'  # 安装指定版本的 pandas 到项目环境
> uv add -r requirements.txt  # 根据 `requirements.txt` 安装非开发依赖
> uv add --dev ipykernel  # 包仅会被加入到开发环境的分组中
> ```

### 2.2 安装 torch-gpu

1. 确认 CUDA 版本以及安装 CUDA 和 cuDNN 版本

> **参考：**[CUDA 和 cuDNN 安装](https://blog.csdn.net/sinat_26398509/article/details/143566753)

2. UV 安装 torch-gpu

> **参考：**[torch-gpu 安装](https://blog.csdn.net/Humbunklung/article/details/146046406)

> ```shell
> # 测试环境
> 显卡：Nvida GeForce RTX 4070 Super
> CUDA 版本：13.1
> CUDA Toolkit 版本 13.0
> cuDNN 版本：9.11.0
> pytorch-gpu 版本：2.9.1
> ```

### 2.3 安装 ultralytics

- 安装 ultralytics 库，测试 yolov8n 预训练参数是否成功加载到 GPU

```shell
uv add ultralytics  # 安装ultralytics库
```

```python
# 测试安装是否全部成功
import torch
import ultralytics
from ultralytics import YOLO

# 验证PyTorch GPU可用性
print("PyTorch GPU可用：", torch.cuda.is_available())
print("CUDA版本：", torch.__version__)
# 验证Ultralytics版本及GPU适配
print("Ultralytics版本：", ultralytics.__version__)
# 加载YOLOv8n并指定GPU
model = YOLO("yolov8n.pt")
model.to("cuda:0")
print("YOLOv8n已加载到GPU：", next(model.model.parameters()).is_cuda)
```

```{figure} _static/{49D7297B-D52E-41BA-A49D-73379241B7F8}.png
:alt: CUDA torch 训练yolov8n模型
:width: 100%
:align: center
```

```python
# 开始训练
results = model.train(
    data="C:/Users/Administrator/Desktop/butter_yolov8n_train/butter_robot.yolov8/data.yaml",   #替换为下载的数据集配置文件路径
    epochs=100, # 训练轮数
    imgsz=640,  # 输入图像大小
    batch=16,   # 批次大小
    device=0,   # 使用默认GPU训练
    project="butter_train_results",  # 训练结果保存目录
    name="yolov8n_butter",  # 训练结果子目录名称
    exist_ok=True  # 允许覆盖已存在的目录
)
```

## 3. Ubuntu 环境实现模型转换

### 3.1 安装 RKNN-Toolkit2

### 3.2 转换模型

### 3.3 仿真推理

### 3.4 ADB 端侧 NPU 推理

### 3.5 交叉编译验证
