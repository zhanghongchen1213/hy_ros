---
name: "ros2-create-pkg-python"
description: "自动创建 ROS2 Python 功能包并完成基础配置，包括 setup.py、package.xml、launch 文件等标准化设置。"
---

# ROS2 Python 功能包创建专家 (ROS2 Python Package Creator)

本技能专注于快速创建和配置 ROS2 Python 功能包，自动完成所有基础配置工作，包括 setup.py、package.xml、launch 文件等，让开发者可以立即开始编写节点逻辑。

## 目标 (Goals)

1. **需求识别 (Requirement Analysis)**：通过交互式问答了解用户需求（包名、节点名、依赖等）。
2. **自动创建 (Auto Creation)**：使用 ros2 pkg create 命令创建标准功能包结构。
3. **配置完善 (Configuration)**：自动配置 setup.py、package.xml、launch 文件等关键文件。
4. **模板生成 (Template Generation)**：生成标准的主节点文件模板，包含基础的 ROS2 节点结构。
5. **验证测试 (Validation)**：编译功能包并验证配置正确性。

## 执行流程 (Execution Flow)

当用户请求"创建 ROS2 Python 功能包"或"新建 ROS2 节点"时：

### 1. 需求收集 (Requirement Collection)

询问用户以下信息：

- **功能包名称** (package_name)：如 `my_robot_control`
- **功能包描述** (description)：如 "机器人运动控制节点"
- **节点名称** (node_name)：如 `control_node`
- **主要依赖** (dependencies)：如 `geometry_msgs`, `sensor_msgs`, `nav_msgs` 等，如果用户不明确，则默认依赖 `rclpy`, `std_msgs`，后面用户可以根据需要添加其他依赖。
- **是否需要自定义参数**：launch 文件中的参数配置

### 2. 创建功能包基础结构 (Create Package Structure)

执行命令：

```bash
# 切换到 ROS2 工作空间的 src 目录
cd <workspace>/src
ros2 pkg create <package_name> \
  --build-type ament_python \
  --dependencies rclpy std_msgs <其他依赖> --License Apache-2.0
```

这将创建以下结构：

```
<package_name>/
├── <package_name>/
│   └── __init__.py
├── resource/
│   └── <package_name>
├── package.xml
├── setup.py
└── setup.cfg
```

### 3. 配置 setup.py (Configure setup.py)

修改 setup.py 文件，添加以下配置：

**3.1 添加 launch 文件配置**
在 `data_files` 列表中添加：

```python
('share/' + package_name + '/launch', ['launch/<package_name>.launch.py']),
```

**3.2 添加 entry_points 配置**

```python
entry_points={
    'console_scripts': [
        '<node_name> = <package_name>.main:main',
    ],
},
```

### 4. 配置 package.xml (Configure package.xml)

修改默认的维护者信息：<maintainer email="k@localhost">k</maintainer> 替换为以下内容

```xml

<maintainer email="k@todo.todo">k</maintainer>

```

在 `<export>` 标签前添加用户指定的依赖：

```xml
<!-- START添加依赖 -->
<exec_depend>依赖包名1</exec_depend>
<exec_depend>依赖包名2</exec_depend>
<!-- END添加依赖 -->
```

标准依赖包括：

- `rclpy`：ROS2 Python 客户端库
- `std_msgs`：标准消息类型
- `launch`：launch 系统
- `launch_ros`：ROS2 launch 扩展

### 5. 创建 launch 文件 (Create Launch File)

**5.1 创建 launch 目录**

```bash
mkdir -p <package_name>/launch
```

**5.2 创建 launch 文件**
创建 `launch/<package_name>.launch.py`，内容模板：

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='<package_name>',
            executable='<node_name>',
            name='<node_name>',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    # 在此添加参数配置
                    # 'param_name': 'param_value',  # 注释
                }
            ],
        )
    ])
```

**关键配置说明**：

- `package`：必须与 setup.py 中的 package_name 一致
- `executable`：必须与 setup.py entry_points 中的脚本名一致
- `name`：节点在 ROS2 网络中的名称
- `output='screen'`：将日志输出到控制台
- `emulate_tty=True`：提升 Python 节点的实时输出表现
- `parameters`：以列表形式传递参数字典

### 6. 创建主节点文件 (Create Main Node File)

创建 `<package_name>/main.py`，提供基础节点模板：

**重要说明**：

- 所有参数应在 launch 文件中声明和传递
- 节点中不使用 `declare_parameter()`，而是直接从 launch 文件读取参数
- 使用辅助函数进行参数类型验证和必需性检查

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class <NodeClassName>(Node):
    """
    <功能包描述>
    """

    def __init__(self):
        super().__init__('<node_name>')

        # 从 launch 文件读取参数（不使用 declare_parameter）
        # 示例：读取字符串参数
        # self.param_str = self._require_str('param_name')
        # 示例：读取整数参数
        # self.param_int = self._require_int('param_name')
        # 示例：读取浮点数参数
        # self.param_float = self._require_float('param_name')

        # 创建发布者
        self.publisher_ = self.create_publisher(
            String,
            'topic_name',
            10
        )

        # 创建订阅者
        self.subscription_ = self.create_subscription(
            String,
            'input_topic',
            self.listener_callback,
            10
        )

        # 创建定时器
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info('<NodeClassName> 节点已启动')

    def _require_str(self, name: str) -> str:
        """读取必需的字符串参数"""
        p = self.get_parameter(name)
        v = p.value
        if isinstance(v, str) and len(v) > 0:
            self.get_logger().info(f"参数 '{name}' 值为 {v}")
            return v
        if v is None or (isinstance(v, str) and len(v) == 0):
            self.get_logger().fatal(f"缺少必需参数 '{name}'（字符串）")
            raise RuntimeError(f"缺少必需参数 '{name}'（字符串）")
        self.get_logger().fatal(
            f"参数 '{name}' 类型不匹配，期望字符串，实际为 {type(v).__name__}"
        )
        raise RuntimeError(f"参数 '{name}' 类型不匹配")

    def _require_int(self, name: str) -> int:
        """读取必需的整数参数"""
        p = self.get_parameter(name)
        v = p.value
        if isinstance(v, int):
            self.get_logger().info(f"参数 '{name}' 值为 {v}")
            return v
        if v is None:
            self.get_logger().fatal(f"缺少必需参数 '{name}'（整数）")
            raise RuntimeError(f"缺少必需参数 '{name}'（整数）")
        self.get_logger().fatal(
            f"参数 '{name}' 类型不匹配，期望整数，实际为 {type(v).__name__}"
        )
        raise RuntimeError(f"参数 '{name}' 类型不匹配")

    def _require_float(self, name: str) -> float:
        """读取必需的浮点数参数"""
        p = self.get_parameter(name)
        v = p.value
        if isinstance(v, float):
            self.get_logger().info(f"参数 '{name}' 值为 {v}")
            return v
        if v is None:
            self.get_logger().fatal(f"缺少必需参数 '{name}'（浮点数）")
            raise RuntimeError(f"缺少必需参数 '{name}'（浮点数）")
        self.get_logger().fatal(
            f"参数 '{name}' 类型不匹配，期望浮点数，实际为 {type(v).__name__}"
        )
        raise RuntimeError(f"参数 '{name}' 类型不匹配")

    def listener_callback(self, msg):
        """订阅回调函数"""
        self.get_logger().info(f'收到消息: {msg.data}')

    def timer_callback(self):
        """定时器回调函数"""
        msg = String()
        msg.data = 'Hello ROS2'
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = <NodeClassName>()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**参数使用说明**：

1. **参数声明位置**：所有参数必须在 launch 文件的 `parameters` 列表中声明
2. **参数读取方式**：在节点 `__init__()` 中使用 `_require_*()` 辅助函数读取
3. **参数类型**：
   - 字符串参数：使用 `self._require_str('param_name')`
   - 整数参数：使用 `self._require_int('param_name')`
   - 浮点数参数：使用 `self._require_float('param_name')`
4. **参数验证**：辅助函数会自动进行类型检查和必需性验证，如果参数缺失或类型不匹配会抛出异常

### 7. 验证配置 (Validation)

**7.1 编译功能包**
在终端中直接运行以下命令，直到验证编译通过，节点正常启动：

```bash
cd <workspace>
colcon build --packages-select <package_name>
```

**7.2 检查安装**

```bash
source install/setup.bash
ros2 pkg list | grep <package_name>
```

**7.3 测试 launch 文件**

```bash
ros2 launch <package_name> <package_name>.launch.py
```

**7.4 测试节点**

```bash
# 终端1：启动节点
ros2 run <package_name> <node_name>

# 终端2：查看话题
ros2 topic list
ros2 topic echo /topic_name
```

## 交互示例 (Example)

**用户输入**：

> "帮我创建一个 ROS2 Python 功能包，用于处理激光雷达数据"

**模型响应**：

1. **需求确认**：
   - 功能包名称：`lidar_processor`
   - 功能包描述：激光雷达数据处理节点
   - 节点名称：`lidar_processor_node`
   - 主要依赖：`sensor_msgs`, `geometry_msgs`
   - 订阅话题：`/scan` (sensor_msgs/LaserScan)
   - 发布话题：`/processed_scan` (sensor_msgs/LaserScan)

2. **执行创建**：
   - 创建功能包基础结构
   - 配置 setup.py（添加 launch 文件和 entry_points）
   - 配置 package.xml（添加 sensor_msgs 和 geometry_msgs 依赖）
   - 创建 launch 文件
   - 创建主节点文件（包含激光雷达数据订阅和处理逻辑）

3. **验证**：
   - 编译功能包
   - 测试 launch 文件启动
   - 确认节点正常运行

## 注意事项 (Notes)

1. **命名规范**：
   - 功能包名使用小写字母和下划线（如 `my_robot_control`）
   - 节点名通常以 `_node` 结尾（如 `control_node`）
   - 类名使用大驼峰命名（如 `MyRobotControl`）

2. **依赖管理**：
   - 在 package.xml 中声明的依赖必须在系统中已安装
   - 常用依赖：`rclpy`, `std_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`
   - 自定义消息包需要单独创建

3. **路径配置**：
   - setup.py 中的 launch 文件路径必须正确
   - entry_points 中的模块路径格式：`<package_name>.main:main`

4. **代码风格**：
   - 遵循 PEP 8 编码规范
   - 使用 flake8 和 pep257 进行代码检查
   - 所有注释和文档使用中文

5. **参数配置**：
   - 使用 `declare_parameter()` 声明参数
   - 在 launch 文件中通过 `parameters` 传递参数
   - 参数名使用小写字母和下划线

6. **日志输出**：
   - 使用 `self.get_logger().info/warn/error()` 输出日志
   - 避免使用 `print()` 函数

## 参考资源 (References)

- **ROS2 官方文档**：https://docs.ros.org/
- **Python 包创建教程**：https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html
- **Launch 文件教程**：https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html
