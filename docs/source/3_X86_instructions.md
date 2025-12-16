# 主机环境配置

## 一、个人初始配置

1. 确保与香橙派处于同一局域网络环境
2. 启动配置 NFS 服务器脚本，挂载本地 NFS 目录: `sudo nfs_config.sh`
3. 将项目文件放到`/home/${SUDO_USER:-$USER}/hy_linux/nfs`目录下
4. 一键安装鱼香 ros2
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

## 二、应用配置

### 2.1 声音复刻模型

```shell
# 安装必要的 python 库
sudo apt-get install python3-aiohttp python3-websockets
```

1. 录制 15s 任意文本音频, 并转换为 base64 编码的 mp3 格式,网页工具https://base64.guru/converter/encode/audio/mp3，下载base64.txt文件,替换hy_ros/src/audio_tts/audio_tts/base64.txt

2. 参考https://ai.baidu.com/ai-doc/REFERENCE/Bkru0l60m 创建应用配置百度智能云 api 密钥，Server Key 更新到 hy_ros/src/audio_tts/launch/audio_tts_launch.py 文件中和 tts_api.py 文件中

3. 在 hy_ros/src/audio_tts/audio_tts 目录下运行`python3 tts_api.py`，查看并记录返回的 Voice ID。更新到 hy_ros/src/audio_tts/launch/audio_tts_launch.py 文件中

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
