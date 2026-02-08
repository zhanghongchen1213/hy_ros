# 🏠 项目主页

# 🛒 购买链接

> 🐟 **闲鱼搜索**：`鸿尘客`
>
> 如有疑问可通过闲鱼私信联系卖家，支持定制化需求沟通。

# 📖 项目简介

**黄油机器人（Butter Robot）** 是一款基于 ROS2 Humble 的全功能自主移动机器人平台，采用 Rockchip RK3588S 作为主控芯片，搭配 ESP32S3 微控制器实现底层运动控制。项目涵盖感知、定位建图、运动控制、视觉处理和语音交互五大核心功能模块，是一个集成度较高的机器人学习与开发平台。

# 📷 产品外观

|                                                                                                  |                                                                                                  |
| :----------------------------------------------------------------------------------------------: | :----------------------------------------------------------------------------------------------: |
| [![yl9J6U.md.png](https://i.imgs.ovh/2026/02/08/yl9J6U.md.png)](https://imgloc.com/image/yl9J6U) | [![yl9EAF.md.png](https://i.imgs.ovh/2026/02/08/yl9EAF.md.png)](https://imgloc.com/image/yl9EAF) |
| [![yl9BC0.md.png](https://i.imgs.ovh/2026/02/08/yl9BC0.md.png)](https://imgloc.com/image/yl9BC0) | [![yl9lw9.md.png](https://i.imgs.ovh/2026/02/08/yl9lw9.md.png)](https://imgloc.com/image/yl9lw9) |
| [![yl9KLc.md.png](https://i.imgs.ovh/2026/02/08/yl9KLc.md.png)](https://imgloc.com/image/yl9KLc) | [![yl9YdX.md.png](https://i.imgs.ovh/2026/02/08/yl9YdX.md.png)](https://imgloc.com/image/yl9YdX) |
| [![yl9GcO.md.png](https://i.imgs.ovh/2026/02/08/yl9GcO.md.png)](https://imgloc.com/image/yl9GcO) | [![yl9m6d.md.png](https://i.imgs.ovh/2026/02/08/yl9m6d.md.png)](https://imgloc.com/image/yl9m6d) |

# 🏗️ 系统架构

系统采用分层模块化架构，主要分为以下五层：

- **感知层**：集成 LDLiDAR 2D 激光雷达（360° 环境扫描）、1080P 摄像头、6 轴 IMU 惯性测量单元和轮式编码器里程计，提供多源环境感知能力。
- **定位与建图层**：基于 slam_toolbox 实现在线异步 SLAM 建图，通过 robot_localization 扩展卡尔曼滤波（EKF）融合轮式里程计与 IMU 数据，并使用 laser_filters 过滤机器人自身遮挡。
- **控制层**：通过 UART 串口（460800 波特率）与 ESP32S3 通信，实现差速驱动底盘的 PID 速度控制和舵机控制，支持键盘、手柄、语音等多种输入方式。
- **视觉处理层**：利用 RK3588S 的 NPU（6TOPS 算力）硬件加速 YOLOv8 目标检测，三个视觉组件（采集、推理、推流）运行在同一 ComposableNodeContainer 中，通过零拷贝通信降低延迟。
- **交互层**：集成 sherpa-onnx ASR 语音识别、RKLLM 本地大语言模型推理和百度大模型音色复刻 TTS 语音合成，实现全流式语音交互；同时通过 Foxglove Bridge 提供 Web 端远程监控与数字孪生可视化。

# 🔗 开源地址

| 平台           | 链接                                                                                   |
| -------------- | -------------------------------------------------------------------------------------- |
| 个人博客       | [https://zhcmqtt.top](https://zhcmqtt.top)                                             |
| CSDN博客       | [https://blog.csdn.net](https://blog.csdn.net/weixin_46477159?spm=1000.2115.3001.5343) |
| 立创硬件开源   | [https://x.jlc.com](https://oshwhub.com/nhwjxzxz/esp32-ping-mu)                        |
| GitHub项目开源 | [https://github.com](https://github.com/zhanghongchen1213/DIY_ESP32_DesktopScreen)     |
| 开发教程开源   | [https://github.com](https://github.com/zhanghongchen1213/DIY_ESP32_DesktopScreen)     |

# 🎬 视频演示

[![点击观看视频](https://i.imgs.ovh/2026/02/08/ylsonn.md.png)](https://ug.link/nas-zhang-GgrM/filemgr/share-download/?id=558c8c6a630d46f0b091d190e990fa9c)

> 📺 **视频作者**：B站 @鸿尘客
> 🔗 **视频链接**：[桌面墨水屏摆件](https://ug.link/nas-zhang-GgrM/filemgr/share-download/?id=558c8c6a630d46f0b091d190e990fa9c)

# ⚙️ 硬件规格

|     项目     |                      参数                      |
| :----------: | :--------------------------------------------: |
|  主芯片 SOC  |                Rockchip RK3588S                |
|     CPU      | 4 核 Cortex-A76(2.4GHZ)+4 核 Cortex-A5(1.8GHZ) |
|     NPU      |                     6TOPS                      |
|     VPU      |          H.264/H.265/JPEG 硬件编解码           |
|     RAM      |                   LPDDR5 8GB                   |
|      TF      |                      32GB                      |
|  Wi-Fi+蓝牙  |                 Wi-Fi 5+BT 5.0                 |
|  驱动器 MCU  |                    ESP32S3                     |
|   电池容量   |                  3000mAh 12V                   |
| 摄像头分辨率 |                1920x1080 30FPS                 |
|   操作系统   |                  Ubuntu22.04                   |
|   ROS 版本   |                  ROS2 Humble                   |

# 📐 硬件框图

[![ylD0Gx.md.png](https://i.imgs.ovh/2026/02/08/ylD0Gx.md.png)](https://imgloc.com/image/ylD0Gx)

# 🚀 产品功能

## 语音交互

系统集成完整的语音交互链路，支持从语音输入到语音输出的全流程处理：

**语音识别（ASR）**

1. 基于 sherpa-onnx 框架，使用 RKNN 加速的流式 Zipformer 模型，支持中英双语实时识别
2. 48kHz 采样输入，内部重采样至 16kHz 处理，支持端点检测（可配置静音超时规则）
3. 增量文本发布机制，识别结果实时输出，无需等待整句完成

**大语言模型（LLM）**

1. 本地部署 Qwen3-VL-2B 模型，W8A8 量化适配 RK3588 NPU，支持 4096 上下文窗口
2. 支持多模态输入（文本 + 图像），可通过触发词唤起摄像头拍照并结合图像进行推理
3. 流式推理输出，通过回调机制将生成结果实时推送至 TTS 模块

**语音合成（TTS）**

1. 接入百度大模型音色复刻 TTS API，支持自定义音色克隆
2. 支持多种方言（四川话、上海话、河南话、湖南话、贵州话等），可调节语速、音调、音量
3. 句级分段合成 + HTTP 流式音频推送（端口 8090），实现边合成边播放的低延迟体验

## 运动控制

**差速驱动底盘**

1. 通过 UART 串口（460800 波特率）与 ESP32S3 通信，采用自定义二进制协议（0xAA 起始帧，0x55/0x66 结束帧）
2. PID 速度环闭环控制，支持实时调参（Kp、Ki、Kd），左右电机独立控制
3. 最大线速度 0.15 m/s，最大角速度 2.0 rad/s

**舵机控制**

1. 3 路舵机（A、B、C）控制机器人脊柱和双臂关节，支持角度精确控制
2. 通过 `/joint_states` 话题发布关节状态，实现 URDF 模型实时联动

**多模式遥控**

1. 键盘控制：WASD 八方向移动，100Hz 轮询频率，0.1 秒无操作自动停车
2. 手柄控制：摇杆模拟量控制线速度和角速度，按键控制舵机角度（±5° 步进）
3. 语音控制：通过语音交互链路下发运动指令

## 视觉交互

**摄像头采集**

1. 基于 GStreamer 硬件加速管线，v4l2src 直接捕获 1920x1080@30fps 画面，输出 NV12 原生格式
2. 支持 RGA 硬件旋转（180°），可通过参数动态开关

**目标检测**

1. YOLOv8 模型部署于 RK3588S NPU（RKNN 加速），640x640 输入，置信度阈值 0.5，NMS 阈值 0.45
2. 支持黄油目标检测、手掌目标检测及自主跟随

**视频推流**

1. 基于 MPP 硬件 H.264 编码，通过 RTSP 协议推流（`rtsp://127.0.0.1:8554/camera`）
2. 三个视觉组件（采集、推理、推流）运行在同一 ComposableNodeContainer 中，启用进程内零拷贝通信，显著降低 CPU 占用和传输延迟

## 定位和导航

**里程计融合**

1. 基于 robot_localization 扩展卡尔曼滤波（EKF），50Hz 频率融合轮式里程计线速度与 IMU 角速度/航向角
2. 2D 模式运行，忽略 Z 轴、横滚和俯仰，输出 `/odometry/filtered` 融合里程计及 odom → base_link TF 变换

**SLAM 建图**

1. 基于 slam_toolbox 在线异步建图，Ceres 求解器优化，地图分辨率 0.05 m/pixel
2. 支持回环检测（搜索距离 3.0 m），最小移动距离 0.1 m 触发扫描匹配
3. 激光雷达最大量程 12.0 m，TF 发布频率 50Hz

**定位模式**

1. slam_toolbox 纯定位模式，加载已有地图进行重定位，不更新地图数据
2. 支持 laser_filters 过滤机器人自身遮挡区域的激光点

**自主导航**

1. 支持基于 Nav2 导航栈的自主导航
2. 支持 DWA 或 TEB 路径规划算法

## 远程监控

1. 集成 Foxglove Bridge（WebSocket 端口 8765），支持 Web 端实时查看机器人状态、话题数据和 TF 坐标树
2. 支持 RTSP 视频流远程查看摄像头画面及目标检测结果
3. 支持 Web 端远程发布控制指令，实现数字孪生效果
