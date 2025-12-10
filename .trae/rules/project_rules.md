# 项目规则（ROS2）

## 参数统一管理（launch为准，源码不设后备）
- 所有运行参数、话题名、节点名、日志级别等，统一在对应包的 `launch/*.launch.py` 中声明与传递。
- 源码不提供后备默认值；参数缺失视为错误，应在节点启动时直接报错并退出。
- 话题名必须通过参数传入，禁止在源码中硬编码（例如 `/uart/chat_gpt_enable`、`/uart/timestamp`）。
- 变更参数只修改 `launch`，不改源码；不同环境可通过不同 `launch` 文件或 `IncludeLaunchDescription` 进行组合。

示例（节点读取）
- 节点构造方式（参考 `src/uart/uart/main.py:34-38`）：
  - 采用 `automatically_declare_parameters_from_overrides=True`，自动从 launch/CLI 声明参数。
  - 严格模式：源码不设默认值；必要参数缺失或类型不匹配直接报错退出。
  - 示例：
    - `super().__init__('uart', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)`
- 必要参数读取方式（参考 `src/uart/uart/main.py:148-169`）：
  - 字符串：`v = self.get_parameter('port').value`，`isinstance(v, str) and len(v) > 0` 校验
  - 整数：`v = self.get_parameter('baud').value`，`isinstance(v, int)` 校验
  - 不符合时输出中文 FATAL 并 `raise RuntimeError`
- 发布/订阅器创建使用参数中的话题名（例如 `chat_topic`、`timestamp_topic`）。

示例（launch 统一传参）
- 参考 `src/uart/launch/uart.launch.py:13-20`，参数容器必须是 list-of-dicts：
  - `parameters=[{'port': '/dev/ttyS3'}, {'baud': 460800}, {'chat_topic': '/uart/chat_gpt_enable'}, {'timestamp_topic': '/uart/timestamp'}]`
  - 禁止使用单个 dict 作为 `parameters`（会触发类型错误）。

## 包结构与安装
- `setup.py` 必须安装 `package.xml` 与 `launch` 目录到 `share/<pkg>/launch`，确保被聚合包检索：
  - `('share/' + package_name, ['package.xml'])`
  - `('share/' + package_name + '/launch', ['launch/<pkg>.launch.py'])`
- `entry_points` 中的控制台脚本名称与 `launch` 中的 `executable` 一致。
- `package.xml` 声明完整的运行依赖与测试依赖（如 `rclpy`、`launch`、`launch_ros`、`ament_index_python`、`python3-serial`）。

## 聚合启动（all_launch）
- 统一聚合包通过 `IncludeLaunchDescription(PythonLaunchDescriptionSource(path))` 引入各功能包的 `launch`。
- 新增功能包需：
  - 在 `all_launch/package.xml` 中添加 `<exec_depend>new_pkg</exec_depend>`。
  - 在该包的 `setup.py` 安装其 `launch` 到 `share/new_pkg/launch`。
  - 在 `all_launch.launch.py` 中加入对应 `IncludeLaunchDescription`。

## 命名与规范
- 包名使用全小写，字母、数字、下划线与短横线；避免大写（如将 `Cerebellum` 规范为 `cerebellum`）。
- 话题与参数命名采用层级命名，例如：`/sensors/uart/chat_gpt_enable`。
- 代码风格遵循 flake8/pep257，测试文件使用 ament_* linters。

## 构建与环境
- 构建前加载发行版环境：`source /opt/ros/$ROS_DISTRO/setup.bash`。
- 构建后加载工作区环境：`source install/setup.bash`。
- 在 NFS 环境下建议将 `build/install/log/test_results` 指向本机可写目录，或在服务器端为特定客户端配置 `no_root_squash`。

## 日志与错误处理
- 日志统一使用中文输出，便于团队理解与现场调试。
- 节点必须在关键路径打印 INFO 级别启动日志与错误日志。
- 发生串口/IO 等异常时应自动重试，并打印中文原因描述。

## 消息与扩展
- 当 payload 字段增多时，优先定义自定义 `msg` 类型统一发布，避免散乱多个话题。
- 参数化开关（如是否发布某些字段）也在 `launch` 中统一配置。

## 测试与校验
- 运行 `colcon test`，保持 linters 通过；若第三方警告可记录并说明原因。
- 所有新增包在 `setup.py` 的 `extras_require['test']` 中包含 `pytest`。


## 脚本结构规范（必须遵循）
- 入口函数：所有脚本必须以 `main()` 为入口；其他逻辑通过函数调用实现。
- 单一职责：每个函数只负责一个明确功能；通过参数与返回值进行通信；禁止隐式全局副作用。
- 工具依赖管理：为每个外部工具实现独立的检测/安装函数，例如：
  - `require_tool_iproute2()`、`require_tool_net_tools()`、`require_tool_nfs_common()`、`install_packages()`。
  - 检测未通过时自动安装，并在安装后进行状态验证（`dpkg -s <pkg>`）。
- 错误处理：统一使用 `set -euo pipefail` 与 `trap 'log_error ...' ERR`；关键操作前进行输入参数校验与失败回滚/退出。
- 日志：统一中文日志，提供 `log_info/log_warn/log_error`；关键步骤均需输出。

### 代码结构示例
```
#!/usr/bin/env bash
set -euo pipefail
trap 'log_error "执行失败：第$LINENO行，退出码=$?"' ERR

ts(){ date '+%F %T'; }
log_info(){ echo "[INFO $(ts)] $*"; }
log_error(){ echo "[ERROR $(ts)] $*" >&2; }

install_packages(){ apt-get update -y && apt-get install -y "$@"; }
require_tool_iproute2(){ command -v ip || install_packages iproute2; }

get_ipv4(){ ip -4 -o addr show up | awk '$2!="lo"{split($4,a,"/"); print a[1]; exit}'; }

main(){
  require_tool_iproute2
  local ip="$(get_ipv4)"
  log_info "本机IP: $ip"
}

main "$@"
```

### 函数文档注释指南
- 每个重要函数需在定义前添加用途与参数/返回值说明（中文）：
```
# 功能：基于 IPv4 生成固定 /24 网段
# 参数：ip（IPv4 字符串）
# 返回：CIDR（例如 192.168.22.0/24）
cidr_24_from_ip(){ ... }
```

### 强制要求
- 本仓库后续新增/修改的所有脚本必须严格遵循以上规范；代码评审以此为准。
