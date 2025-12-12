# 1.新增：指定 C++ 标准为 C++14
cmake_minimum_required(VERSION 3.14)
project(audio_rkllm)

# 2.新增：默认使用 C++14 标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 开启编译器警告
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 3.新增：寻找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 4.新增：包含 src 目录以支持同目录下的头文件
include_directories(include src)


# 5.新增：创建可执行文件：将 .h 文件也加入源文件列表
add_executable(audio_rkllm_node
  src/audio_rkllm.cpp
  src/audio_rkllm.h
)

# 6.新增：链接依赖库
ament_target_dependencies(audio_rkllm_node
  rclcpp
  std_msgs
)

# 7.新增：安装可执行文件
install(TARGETS audio_rkllm_node
  DESTINATION lib/${PROJECT_NAME}
)

# 8.新增：安装 launch 启动文件目录
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

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
