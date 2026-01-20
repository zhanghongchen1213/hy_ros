---
name: "ros-node-optimization"
description: "分析 ROS 节点资源占用，移除冗余依赖，优化代码以降低 CPU/内存占用，生成建议并自动应用修复。当用户要求优化 ROS 节点性能或轻量化时调用。"
---

# ROS 节点轻量化与性能优化 (ROS Node Lightweight & Optimization)

本技能专注于对 ROS/ROS2 节点进行深度代码分析与资源审计，通过移除冗余依赖、优化消息传递机制、改进回调处理逻辑，来降低 CPU 和内存占用，实现节点的轻量化。

## 目标 (Goals)

1.  **依赖清理 (Dependency Pruning)**：分析 `package.xml` 和 `CMakeLists.txt`，移除未被源代码实际引用的包依赖。
2.  **通信优化 (Communication Optimization)**：
    - 检测并推荐使用 **Zero-Copy**（零拷贝）传输（如 ROS2 的 `loaned messages` 或 C++ `const &` 引用传递）。
    - 建议将独立节点转换为 **Nodelets** (ROS1) 或 **Components** (ROS2) 以实现进程内通信。
3.  **代码效能 (Code Efficiency)**：
    - 识别高频回调中的耗时操作（如 I/O、内存分配）。
    - 检查回调队列（Callback Queue）配置，避免阻塞。
    - 推荐使用 **内存池 (Memory Pool)** 和 **自定义分配器** 避免运行时内存分配。
4.  **并发调度 (Concurrency Scheduling)**：
    - 分析 **Executor** 模型（Single vs Multi-threaded）。
    - 优化 **Callback Group** 配置（Reentrant vs Mutually Exclusive），分离实时任务与耗时任务。
5.  **自动修复 (Auto-Fix)**：自动修改构建文件移除依赖，重构代码应用优化模式。

## 执行流程 (Execution Flow)

当用户请求“优化 ROS 节点”、“减少 CPU/内存占用”或“ROS 项目轻量化”时：

### 1. 依赖与构建分析 (Dependency Analysis)

- **扫描引用**：遍历 `src/` 和 `include/`，提取所有 `#include` 头文件。
- **对比配置**：对比 `package.xml` 中的 `<depend>` 标签，找出声明了但未使用的 ROS 包。
- **动作**：生成移除建议，并修改 `package.xml` 和 `CMakeLists.txt`。

### 2. 性能模式匹配 (Performance Pattern Matching)

- **消息传递检查**：
  - _低效模式_：`void callback(std_msgs::String msg)` （值传递，产生拷贝）。
  - _优化建议_：
    - ROS2: `void callback(const std_msgs::String::SharedPtr msg)` 或使用 `UniquePtr` 发布。
    - ROS2 IPC: 推荐使用 `borrow_loaned_message()` 进行零拷贝发布。
    - ROS1: `const std_msgs::String::ConstPtr& msg`。
- **并发模型检查**：
  - 检查是否使用了 `MultiThreadedExecutor` 但未配置 `CallbackGroup`（导致实际仍串行）。
  - 建议将耗时处理（图像/点云）放入 `Reentrant` 回调组。
- **内存管理**：
  - 检查回调函数中是否存在 `new`/`malloc` 或 `std::vector` 的反复创建。
  - 建议使用成员变量预分配或 `reserve()` 内存。
- **发布频率检查**：
  - 检查 `ros::Rate` 或 `create_wall_timer` 的频率参数，提示过高频率（>100Hz）的必要性。

### 3. 生成报告与执行 (Report & Execute)

- **输出报告**：
  - **冗余依赖**：列出将要移除的包。
  - **代码热点**：指出具体的函数行号及优化理由。
  - **架构建议**：建议合并节点为 Component Container；建议分离高低频回调组。
- **执行修复**：
  - 自动清理 `package.xml`。
  - 自动重构回调函数签名。
  - 替换大对象的拷贝操作。

## 交互示例 (Example)

**用户输入**：

> "帮我优化一下这个 perception_node，感觉 CPU 占用太高了，而且有丢帧。"

**模型响应**：

1.  **分析**：
    - 发现 `package.xml` 依赖了 `pcl_ros` 和 `opencv`，但代码中只用了 `opencv`。
    - 发现点云回调 `void cloud_cb(sensor_msgs::PointCloud2 cloud)` 使用了值传递。
    - 发现该节点使用了默认的 `SingleThreadedExecutor`，导致图像处理阻塞了里程计更新。
2.  **报告**：
    > ### 优化方案
    >
    > 1. **移除依赖**：移除 `pcl_ros`。
    > 2. **零拷贝优化**：修改 `cloud_cb` 为 `SharedPtr` 引用传递。
    > 3. **并发优化**：将图像回调放入 `Reentrant CallbackGroup`，并建议使用 `MultiThreadedExecutor` 启动。
3.  **执行**：
    - 修改 `package.xml` 删除 `<depend>pcl_ros</depend>`。
    - 修改源码：重构回调签名，添加 CallbackGroup 定义。

## 注意事项 (Notes)

- **ROS 版本差异**：明确区分 ROS1 (Nodelet, ConstPtr) 和 ROS2 (Component, SharedPtr, LoanedMessage) 的 API 差异。
- **实时性**：对于实时性要求高的节点，避免在回调中进行 `new/malloc`，建议预分配内存或使用内存池。
- **线程安全**：在使用 `MultiThreadedExecutor` 和 `Reentrant` 回调组时，务必检查共享变量的锁保护（`std::mutex`）。
