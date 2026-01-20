---
name: "ros2-create-pkg-cmake"
description: "自动创建 ROS2 CMake 功能包并完成基础配置，包括 package.xml、CMakeLists.txt、launch 文件和源代码模板。"
---

# ROS2 CMake 功能包创建专家 (ROS2 CMake Package Creator)

本技能专注于快速创建和配置 ROS2 CMake 功能包，自动完成所有基础配置工作，包括 package.xml、CMakeLists.txt、launch 文件和源代码模板，让开发者可以立即开始编写节点逻辑。

## 目标 (Goals)

1. **需求识别 (Requirement Analysis)**：通过交互式问答了解用户需求（包名、节点名、依赖等）。
2. **自动创建 (Auto Creation)**：使用 ros2 pkg create 命令创建标准功能包结构。
3. **配置完善 (Configuration)**：自动配置 package.xml 和 CMakeLists.txt 关键文件。
4. **Launch文件生成 (Launch File Generation)**：生成标准的 Python launch 文件，包含参数配置模板。
5. **源码模板生成 (Source Template Generation)**：生成最小化的 C++ 节点模板（.cpp 和 .h 文件）。
6. **编译验证 (Build Validation)**：编译功能包并验证配置正确性。

## 执行流程 (Execution Flow)

当用户请求"创建 ROS2 CMake 功能包"或"新建 ROS2 C++ 节点"时：

### 1. 需求收集 (Requirement Collection)

询问用户以下信息：

- **功能包名称** (package_name)：如 `robot_navigation`
- **功能包描述** (description)：如 "机器人导航控制节点"
- **节点名称** (node_name)：如 `navigation_node`
- **主要依赖** (dependencies)：如 `geometry_msgs`, `sensor_msgs`, `nav_msgs` 等，如果用户不明确，则默认依赖 `rclcpp`, `std_msgs`，后面用户可以根据需要添加其他依赖。

### 2. 创建功能包基础结构 (Create Package Structure)

执行命令：

```bash
# 切换到 ROS2 工作空间的 src 目录
cd <workspace>/src
ros2 pkg create <package_name> \
  --build-type ament_cmake \
  --dependencies rclcpp std_msgs <其他依赖> \
  --license Apache-2.0
```

这将创建以下结构：

```
<package_name>/
├── include/
│   └── <package_name>/
├── src/
├── CMakeLists.txt
└── package.xml
```

### 3. 配置 package.xml (Configure package.xml)

修改 package.xml 文件，确保包含以下内容：

**3.1 维护者信息**
修改默认的维护者信息：<maintainer email="k@localhost">k</maintainer> 替换为以下内容

```xml

<maintainer email="k@todo.todo">k</maintainer>

```

**3.2 添加依赖**
在 `<export>` 标签前添加用户指定的依赖：

```xml
<buildtool_depend>ament_cmake</buildtool_depend>

<depend>rclcpp</depend>
<depend>std_msgs</depend>
<depend>launch</depend>
<depend>launch_ros</depend>
<depend>ament_lint_auto</depend>
<depend>ament_lint_common</depend>

<!-- 根据用户需求添加其他依赖 -->
<!-- <depend>geometry_msgs</depend> -->
<!-- <depend>sensor_msgs</depend> -->

<test_depend>ament_lint_auto</test_depend>
<test_depend>ament_lint_common</test_depend>

<export>
  <build_type>ament_cmake</build_type>
</export>
```

### 4. 配置 CMakeLists.txt (Configure CMakeLists.txt)

修改 CMakeLists.txt 文件，按照以下 11 个部分进行配置：

**4.1 指定 CMake 版本和项目名**

```cmake
# 1.新增：指定 C++ 标准为 C++14（必选）
cmake_minimum_required(VERSION 3.14)
project(<package_name>)
```

**4.2 设置 C++ 标准**

```cmake
# 2.新增：默认使用 C++14 标准（必选）
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
```

**4.3 开启编译器警告**

```cmake
# 3.新增：开启编译器警告（可选）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
```

**4.4 寻找依赖包**

```cmake
# 4.新增：寻找依赖包（必选）
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# 根据用户需求添加其他依赖
# find_package(geometry_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)
```

**4.5 包含目录**

```cmake
# 5.新增：包含 src 目录以支持同目录下的头文件（必选）
include_directories(include src)
```

**4.6 创建可执行文件**

```cmake
# 6.新增：创建可执行文件：将 .h 文件也加入源文件列表（必选）
add_executable(<node_name>
  src/<package_name>.cpp
  src/<package_name>.h
)
```

**4.7 链接外部库（可选）**

```cmake
# 7.新增：链接外部库（可选）
# 如果需要链接外部库，可以使用以下方式：
# set(EXTERNAL_LIB_PATH /path/to/library.so)
# target_link_libraries(<node_name>
#   ${EXTERNAL_LIB_PATH}
# )
```

**4.8 链接 ROS 依赖库**

```cmake
# 8.新增：链接依赖库（必选）
ament_target_dependencies(<node_name>
  rclcpp
  std_msgs
)
```

**4.9 安装可执行文件**

```cmake
# 9.新增：安装可执行文件（必选）
install(TARGETS <node_name>
  DESTINATION lib/${PROJECT_NAME}
)
```

**4.10 安装 launch 文件**

```cmake
# 10.新增：安装 launch 启动文件目录（必选）
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
```

**4.11 测试配置**

```cmake
# 测试相关配置
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 跳过版权检查
  set(ament_cmake_copyright_FOUND TRUE)
  # 跳过 cpplint 检查
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

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
            package='<package_name>',  # 所属包名
            executable='<node_name>',  # 可执行文件名
            name='<node_name>',  # ROS 网络中的节点名称
            output='screen',  # 将节点日志输出到控制台
            emulate_tty=True,  # 提升实时输出表现
            parameters=[  # ROS2 要求为 list；内部为单个 dict 容器
                {
                    # 在此添加参数配置
                    # 'param_name': 'param_value',  # 参数说明
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

### 6. 生成源代码文件 (Create Source Files)

**重要说明**：

- 所有参数应在 launch 文件中声明和传递
- 节点中不使用 `declare_parameter()`，而是直接从 launch 文件读取参数
- 使用辅助函数进行参数类型验证和必需性检查

**6.1 创建头文件**
创建 `src/<package_name>.h`，提供节点模板：

```cpp
#ifndef <PACKAGE_NAME>_H_
#define <PACKAGE_NAME>_H_

#include "rclcpp/rclcpp.hpp"
#include <string>

class <NodeClassName> : public rclcpp::Node
{
public:
    <NodeClassName>();
    ~<NodeClassName>();

private:
    // 参数读取辅助函数
    std::string require_string(const std::string& name);
    int require_int(const std::string& name);
    double require_double(const std::string& name);

    // 在此添加私有成员变量和函数
};

#endif  // <PACKAGE_NAME>_H_
```

**6.2 创建实现文件**
创建 `src/<package_name>.cpp`，提供节点实现：

```cpp
#include "<package_name>.h"

<NodeClassName>::<NodeClassName>() : Node("<node_name>")
{
    // 从 launch 文件读取参数（不使用 declare_parameter）
    // 示例：读取字符串参数
    // std::string param_str = require_string("param_name");
    // 示例：读取整数参数
    // int param_int = require_int("param_name");
    // 示例：读取浮点数参数
    // double param_double = require_double("param_name");

    RCLCPP_INFO(this->get_logger(), "<NodeClassName> 节点已启动");
}

<NodeClassName>::~<NodeClassName>()
{
    RCLCPP_INFO(this->get_logger(), "<NodeClassName> 节点已关闭");
}

// 参数读取辅助函数实现
std::string <NodeClassName>::require_string(const std::string& name)
{
    rclcpp::Parameter param = this->get_parameter(name);
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        std::string value = param.as_string();
        if (!value.empty()) {
            RCLCPP_INFO(this->get_logger(), "参数 '%s' 值为 %s", name.c_str(), value.c_str());
            return value;
        }
    }
    RCLCPP_FATAL(this->get_logger(), "缺少必需参数 '%s'（字符串）", name.c_str());
    throw std::runtime_error("缺少必需参数: " + name);
}

int <NodeClassName>::require_int(const std::string& name)
{
    rclcpp::Parameter param = this->get_parameter(name);
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        int value = param.as_int();
        RCLCPP_INFO(this->get_logger(), "参数 '%s' 值为 %d", name.c_str(), value);
        return value;
    }
    RCLCPP_FATAL(this->get_logger(), "缺少必需参数 '%s'（整数）", name.c_str());
    throw std::runtime_error("缺少必需参数: " + name);
}

double <NodeClassName>::require_double(const std::string& name)
{
    rclcpp::Parameter param = this->get_parameter(name);
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        double value = param.as_double();
        RCLCPP_INFO(this->get_logger(), "参数 '%s' 值为 %f", name.c_str(), value);
        return value;
    }
    RCLCPP_FATAL(this->get_logger(), "缺少必需参数 '%s'（浮点数）", name.c_str());
    throw std::runtime_error("缺少必需参数: " + name);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<<NodeClassName>>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

**参数使用说明**：

1. **参数声明位置**：所有参数必须在 launch 文件的 `parameters` 列表中声明
2. **参数读取方式**：在节点构造函数中使用辅助函数读取
3. **参数类型**：
   - 字符串参数：使用 `require_string("param_name")`
   - 整数参数：使用 `require_int("param_name")`
   - 浮点数参数：使用 `require_double("param_name")`
4. **参数验证**：辅助函数会自动进行类型检查和必需性验证，如果参数缺失或类型不匹配会抛出异常

### 7. 编译验证 (Build Validation)

在终端中直接运行以下命令，直到验证编译通过，节点正常启动：

**7.1 编译功能包**

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

# 终端2：查看节点信息
ros2 node list
ros2 node info /<node_name>
```

## 交互示例 (Example)

**用户输入**：

> "帮我创建一个 ROS2 CMake 功能包，用于机器人导航控制"

**模型响应**：

1. **需求确认**：
   - 功能包名称：`robot_navigation`
   - 功能包描述：机器人导航控制节点
   - 节点名称：`navigation_node`
   - 主要依赖：`geometry_msgs`, `nav_msgs`

2. **执行创建**：
   - 创建功能包基础结构
   - 配置 package.xml（添加 geometry_msgs 和 nav_msgs 依赖）
   - 配置 CMakeLists.txt（添加 11 个标准配置段）
   - 创建 launch 文件
   - 创建源代码文件（robot_navigation.h 和 robot_navigation.cpp）

3. **验证**：
   - 编译功能包
   - 测试 launch 文件启动
   - 确认节点正常运行

## 注意事项 (Notes)

1. **命名规范**：
   - 功能包名使用小写字母和下划线（如 `robot_navigation`）
   - 节点名通常以 `_node` 结尾（如 `navigation_node`）
   - 类名使用大驼峰命名（如 `RobotNavigation`）
   - 头文件宏定义使用大写字母和下划线（如 `ROBOT_NAVIGATION_H_`）

2. **依赖管理**：
   - 在 package.xml 中声明的依赖必须在系统中已安装
   - 常用依赖：`rclcpp`, `std_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`
   - 自定义消息包需要单独创建
   - CMakeLists.txt 中的 find_package 必须与 package.xml 中的依赖一致

3. **目录结构**：
   - 头文件放在 `include/<package_name>/` 或 `src/` 目录
   - 实现文件放在 `src/` 目录
   - launch 文件放在 `launch/` 目录
   - 如果头文件在 `src/` 目录，需要在 CMakeLists.txt 中添加 `include_directories(src)`

4. **可执行文件命名**：
   - CMakeLists.txt 中的可执行文件名（add_executable）通常以 `_node` 结尾
   - launch 文件中的 executable 参数必须与 CMakeLists.txt 中的可执行文件名一致
   - 节点名称（Node 构造函数参数）可以与可执行文件名不同

5. **CMake 配置**：
   - 必须按照 11 个标准段的顺序配置 CMakeLists.txt
   - 所有中文注释必须使用 UTF-8 编码
   - 测试配置中跳过 copyright 和 cpplint 检查可以加快编译速度

6. **Launch 文件配置**：
   - launch 文件必须以 `.launch.py` 结尾
   - package 参数必须与功能包名一致
   - executable 参数必须与 CMakeLists.txt 中的可执行文件名一致
   - parameters 参数必须是列表形式，内部包含字典

7. **编译系统**：
   - 使用 `colcon build --packages-select <package_name>` 单独编译功能包
   - 编译后必须 source install/setup.bash 才能使用
   - 修改代码后需要重新编译
   - 修改 launch 文件不需要重新编译

8. **常见错误**：
   - 找不到头文件：检查 include_directories 配置
   - 链接错误：检查 ament_target_dependencies 配置
   - launch 文件找不到：检查 install(DIRECTORY launch) 配置
   - 节点无法启动：检查 executable 名称是否与 CMakeLists.txt 一致

## 参考资源 (References)

- **ROS2 官方文档**：https://docs.ros.org/
- **CMake 包创建教程**：https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html
