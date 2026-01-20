---
name: "ros2-pkg-docs"
description: "分析指定的 ROS2 功能包结构、代码和配置，自动生成包含节点详情、接口定义、参数说明、依赖关系及使用示例的专用技术文档。"
---

# ROS2 功能包文档生成专家 (ROS2 Package Documentation Generator)

本技能专注于分析单个 ROS2 功能包，深入解读其源代码、配置文件和接口定义，生成详尽的技术文档。

## 目标 (Goals)

1. **结构分析 (Structure Analysis)**：解析功能包的目录结构、构建类型（Python/CMake）和文件组成。
2. **节点解析 (Node Parsing)**：识别所有节点，提取发布/订阅话题、服务、动作和参数。
3. **接口文档 (Interface Documentation)**：解析自定义消息 (`msg`)、服务 (`srv`) 和动作 (`action`) 定义。
4. **依赖梳理 (Dependency Analysis)**：分析 `package.xml` 中的依赖关系。
5. **使用说明 (Usage Guide)**：生成编译、运行和测试的完整说明。

## 执行流程 (Execution Flow)

当用户请求"生成功能包文档"或"分析 ROS2 包"时：

### 1. 功能包定位 (Package Location)

询问用户或自动检测：

- **功能包路径**：如 `src/my_robot_control`
- **功能包名称**：从 `package.xml` 中读取

### 2. 基础信息收集 (Basic Information Collection)

读取并解析以下文件：

```
<package_path>/
├── package.xml          # 包元信息、依赖
├── setup.py             # Python 包配置（如适用）
├── setup.cfg            # Python 配置（如适用）
├── CMakeLists.txt       # CMake 配置（如适用）
├── <package_name>/      # Python 源码目录
│   ├── __init__.py
│   └── *.py             # 节点源码
├── src/                 # C++ 源码目录（如适用）
├── include/             # C++ 头文件（如适用）
├── launch/              # Launch 文件
├── config/              # 配置文件（YAML 等）
├── msg/                 # 自定义消息
├── srv/                 # 自定义服务
└── action/              # 自定义动作
```

### 3. 节点深度分析 (Node Deep Analysis)

对每个节点文件进行代码分析：

**3.1 提取节点类信息**
- 节点名称（`super().__init__('node_name')`）
- 类名和继承关系

**3.2 提取通信接口**
- **发布者 (Publishers)**：话题名、消息类型、QoS 设置
- **订阅者 (Subscribers)**：话题名、消息类型、回调函数
- **服务服务器 (Service Servers)**：服务名、服务类型
- **服务客户端 (Service Clients)**：服务名、服务类型
- **动作服务器 (Action Servers)**：动作名、动作类型
- **动作客户端 (Action Clients)**：动作名、动作类型
- **定时器 (Timers)**：周期、回调函数

**3.3 提取参数 (Parameters)**
- 参数名称
- 参数类型
- 默认值
- 参数描述（从注释或 `declare_parameter` 中提取）

### 4. Launch 文件分析 (Launch File Analysis)

解析 launch 文件：
- 启动的节点列表
- 节点参数配置
- 条件启动逻辑
- 包含的其他 launch 文件

### 5. 文档生成 (Document Generation)

## 文档模板

```markdown
# [功能包名称] 技术文档

> **包名**: `<package_name>`
> **版本**: `<version>`
> **构建类型**: `ament_python` / `ament_cmake`
> **维护者**: `<maintainer>`
> **许可证**: `<license>`

## 1. 概述 (Overview)

[功能包描述，从 package.xml 的 description 字段提取]

## 2. 目录结构 (Directory Structure)

```
<package_name>/
├── package.xml
├── setup.py / CMakeLists.txt
├── <package_name>/
│   ├── __init__.py
│   ├── main.py           # 主节点
│   └── utils.py          # 工具模块
├── launch/
│   └── <package_name>.launch.py
├── config/
│   └── params.yaml
└── msg/
    └── CustomMsg.msg
```

## 3. 依赖关系 (Dependencies)

### 3.1 构建依赖 (Build Dependencies)

| 依赖包 | 说明 |
|--------|------|
| `rclpy` | ROS2 Python 客户端库 |
| `std_msgs` | 标准消息类型 |

### 3.2 执行依赖 (Exec Dependencies)

| 依赖包 | 说明 |
|--------|------|
| `sensor_msgs` | 传感器消息类型 |

## 4. 节点详解 (Node Details)

### 4.1 [节点名称] (`<node_name>`)

**源文件**: `<package_name>/main.py`
**可执行文件**: `<executable_name>`

#### 功能描述

[节点的主要功能和用途]

#### 发布话题 (Published Topics)

| 话题名 | 消息类型 | 频率 | 描述 |
|--------|----------|------|------|
| `/output` | `std_msgs/String` | 10Hz | 处理结果输出 |

#### 订阅话题 (Subscribed Topics)

| 话题名 | 消息类型 | 描述 |
|--------|----------|------|
| `/input` | `std_msgs/String` | 输入数据 |

#### 提供的服务 (Services)

| 服务名 | 服务类型 | 描述 |
|--------|----------|------|
| `/reset` | `std_srvs/Trigger` | 重置节点状态 |

#### 参数 (Parameters)

| 参数名 | 类型 | 默认值 | 描述 |
|--------|------|--------|------|
| `update_rate` | `float` | `10.0` | 更新频率 (Hz) |
| `enable_debug` | `bool` | `false` | 是否启用调试模式 |

## 5. 自定义接口 (Custom Interfaces)

### 5.1 消息 (Messages)

#### `CustomMsg.msg`

```
# 自定义消息描述
Header header
float64 value
string status
```

| 字段 | 类型 | 描述 |
|------|------|------|
| `header` | `std_msgs/Header` | 消息头 |
| `value` | `float64` | 数值数据 |
| `status` | `string` | 状态信息 |

### 5.2 服务 (Services)

#### `CustomSrv.srv`

```
# 请求
int32 command
---
# 响应
bool success
string message
```

## 6. Launch 文件 (Launch Files)

### 6.1 `<package_name>.launch.py`

**启动命令**:
```bash
ros2 launch <package_name> <package_name>.launch.py
```

**启动参数**:

| 参数名 | 默认值 | 描述 |
|--------|--------|------|
| `use_sim_time` | `false` | 是否使用仿真时间 |

**启动的节点**:
- `<package_name>/<node_name>`

## 7. 配置文件 (Configuration Files)

### 7.1 `config/params.yaml`

```yaml
<node_name>:
  ros__parameters:
    update_rate: 10.0
    enable_debug: false
```

## 8. 使用指南 (Usage Guide)

### 8.1 编译 (Build)

```bash
cd <workspace>
colcon build --packages-select <package_name>
source install/setup.bash
```

### 8.2 运行 (Run)

**使用 Launch 文件启动**:
```bash
ros2 launch <package_name> <package_name>.launch.py
```

**单独运行节点**:
```bash
ros2 run <package_name> <node_name>
```

### 8.3 测试 (Test)

```bash
# 查看节点列表
ros2 node list

# 查看话题
ros2 topic list
ros2 topic echo /output

# 调用服务
ros2 service call /reset std_srvs/srv/Trigger
```

## 9. 常见问题 (Troubleshooting)

### 问题 1: 节点启动失败

**现象**: `ModuleNotFoundError: No module named '<package_name>'`

**解决方案**:
1. 确保已编译功能包: `colcon build --packages-select <package_name>`
2. 确保已 source 环境: `source install/setup.bash`

### 问题 2: 话题无数据

**现象**: `ros2 topic echo` 无输出

**排查步骤**:
1. 检查节点是否运行: `ros2 node list`
2. 检查话题是否存在: `ros2 topic list`
3. 检查发布频率: `ros2 topic hz /topic_name`

## 10. API 参考 (API Reference)

### 10.1 类: `<NodeClassName>`

```python
class <NodeClassName>(Node):
    """
    [类文档字符串]
    """

    def __init__(self):
        """初始化节点"""
        pass

    def callback(self, msg):
        """
        回调函数描述

        Args:
            msg: 消息对象
        """
        pass
```

## 11. 更新日志 (Changelog)

| 版本 | 日期 | 更新内容 |
|------|------|----------|
| 1.0.0 | YYYY-MM-DD | 初始版本 |
```

## 注意事项 (Notes)

1. **代码分析范围**：
   - 仅分析 Python 节点时，解析 `*.py` 文件
   - 分析 C++ 节点时，解析 `*.cpp` 和 `*.hpp` 文件
   - 使用 AST 或正则表达式提取关键信息

2. **接口识别模式**：
   - 发布者：`create_publisher(`
   - 订阅者：`create_subscription(`
   - 服务服务器：`create_service(`
   - 服务客户端：`create_client(`
   - 定时器：`create_timer(`
   - 参数：`declare_parameter(` 或 `get_parameter(`

3. **文档输出位置**：
   - 默认输出到 `<package_path>/docs/README.md`
   - 可选输出到用户指定位置

4. **语言规范**：
   - 所有文档内容使用中文
   - 代码示例保持原样
   - 技术术语可使用英文原文

## 交互示例 (Example)

**用户输入**：

> "帮我分析 src/lidar_processor 功能包并生成文档"

**模型响应**：

1. **定位功能包**：
   - 路径：`src/lidar_processor`
   - 确认包含 `package.xml`

2. **分析内容**：
   - 读取 `package.xml` 获取元信息
   - 分析 `lidar_processor/main.py` 节点代码
   - 解析 `launch/lidar_processor.launch.py`
   - 检查是否有自定义消息

3. **生成文档**：
   - 创建 `docs/README.md`
   - 包含完整的节点接口表格
   - 生成使用指南和示例命令

4. **输出结果**：
   - 显示文档摘要
   - 提供文档路径

## 参考资源 (References)

- **ROS2 开发者文档**：https://docs.ros.org/en/humble/
- **包格式规范**：https://www.ros.org/reps/rep-0149.html
- **接口定义**：https://docs.ros.org/en/humble/Concepts/About-ROS-Interfaces.html
