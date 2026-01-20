# 基础编程规则 (Basic Programming Rules) - 场景B: ROS 功能包

## 🎯 核心目标
本文件定义了 **ROS 2 开发场景**下的项目“宪法”。确立基于 **REP 2004** (Package Quality Categories) 和 **Google ROS Style** 的编码规范，确保功能包具备模块化、可复用性和社区兼容性。

## 💡 核心理念
**模块化，异步优先 (Modularity, Asynchrony First)**。
ROS 系统的核心是节点间通信。我们优先选择**松耦合**的设计、**非阻塞**的回调处理和**标准消息接口**。

## 📝 规则详情

### 1. 项目结构规范 (Project Structure)
遵循 `ament_cmake` 或 `ament_python` 标准结构：
```text
my_ros_package/
├── package.xml         # 包元数据
├── CMakeLists.txt      # (C++) 构建脚本
├── setup.py            # (Python) 安装脚本
├── include/            # (C++) 头文件
│   └── my_ros_package/
├── src/                # (C++) 源文件
├── my_ros_package/     # (Python) 模块包
│   └── __init__.py
├── launch/             # 启动文件 (.launch.py)
├── config/             # 配置文件 (.yaml)
├── msg/                # 自定义消息 (.msg)
├── srv/                # 自定义服务 (.srv)
└── test/               # 测试代码
```
- **规则**: 严禁将 C++ 和 Python 源码混杂在同一级目录。必须分离 `include` 和 `src`。

### 2. 技术栈与语言限制 (Tech Stack Constraints)
- **ROS 版本**: ROS 2 (Humble/Rolling)。
- **语言标准**: C++17 (C++14 minimum) / Python 3.10+。
- **构建系统**: Colcon (ament_cmake / ament_python)。
- **节点设计 (Node Design)**:
    - ✅ **继承式**: 继承 `rclcpp::Node` 或 `rclpy.node.Node`。
    - ✅ **组件化**: C++ 节点应编写为 `Component`，支持运行时动态加载 (Composition)。
- **并发模型**:
    - 严禁在回调函数中执行长耗时操作（如 `sleep` 或复杂计算）。
    - 耗时任务必须移至 Action Server 或独立线程。

### 3. 命名规则 (Naming Conventions)
遵循 **ROS 2 Style Guide**:
- **包名**: `snake_case` (e.g., `navigation_utils`)。
- **节点名**: `snake_case` (e.g., `planner_server`)。
- **话题/服务/动作**: `snake_case` (e.g., `/camera/image_raw`, `/compute_path`)。
- **C++**:
    - 类名: `PascalCase`
    - 方法/变量: `snake_case`
    - 成员变量: 结尾加下划线 `variable_`
- **Python**: 遵循 PEP 8。

### 4. 代码风格指南 (Style Guide)
- **C++**: 遵循 `Google C++ Style Guide` + ROS 扩展。
    - 使用 `std::shared_ptr` 和 `std::unique_ptr` 管理资源。
    - 使用 `RCLCPP_INFO` 代替 `std::cout`。
- **Python**: 遵循 `PEP 8`。
    - 使用 `self.get_logger().info()` 代替 `print`。
- **Linter 强制**:
    - C++: `ament_cpplint`, `ament_uncrustify`.
    - Python: `ament_flake8`, `ament_pep257`.

### 5. Git 工作流规范 (Git Workflow)
- **分支**:
    - `humble-devel`: 针对 Humble 发行版的主分支。
    - `rolling-devel`: 针对 Rolling 发行版的主分支。
- **提交信息**:
    - `feat(nav): add A* planner plugin`
    - `fix(driver): resolve laser scan timeout`

### 6. 文档管理
- **README.md**: 必须包含节点参数表、订阅/发布的话题列表、启动指令。
- **Launch 文件**: 提供带参数说明的 `launch.py`。
- **API 文档**: 使用 ROS Doc 规范。
