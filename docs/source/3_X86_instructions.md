# 主机环境配置

## 一、个人初始配置

1. 确保与香橙派处于同一局域网络环境
2. 启动配置NFS服务器脚本，挂载本地NFS目录: `sudo nfs_config.sh`
3. 将项目文件放到`/home/${SUDO_USER:-$USER}/hy_linux/nfs`目录下
4. 一键安装鱼香ros2
    - 参考鱼香ros一键安装教程安装ros2
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
    - 可在主机环境按照相同方法同步配置完ros2后，测试双端通信是否成功
        - 香橙派端侧：`ros2 run demo_nodes_cpp talker`
        - 主机侧：`ros2 run demo_nodes_cpp listener`
    ```{figure} _static/2D377DF7-6518-45F7-B95B-6FD6E9846352.png
    :alt: 通信测试
    :width: 100%
    :align: center
    ```

## 二、应用配置