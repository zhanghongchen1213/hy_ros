# 基础编程规则 (Basic Programming Rules) - 场景 A: 嵌入式开发

## 🎯 核心目标

本文件定义了**嵌入式开发场景**下的项目“宪法”。确立统一的 C/C++ 代码风格、内存管理策略、硬件抽象层级和并发安全规范，确保代码符合 **MISRA C:2023** 标准，具备高可靠性和可移植性。

## 💡 核心理念

**安全至上，确定性优先 (Safety First, Determinism First)**。
在嵌入式系统中，资源受限且故障成本高昂。我们优先选择**静态分配**、**明确的类型**和**可预测的行为**，而非灵活性。

## 📝 规则详情

### 1. 项目结构规范 (Project Structure)

采用分层架构，严格隔离硬件依赖：

```text
src/
├── app/            # 应用层逻辑 (Application Layer, 平台无关)
│   ├── main.c      # 入口函数
│   └── logic/      # 业务状态机
├── middleware/     # 中间件 (Middleware: RTOS, Filesystem, USB stack)
├── drivers/        # 驱动层 (Drivers: HAL, BSP)
│   ├── bsp/        # 板级支持包 (Board Support Package)
│   └── hal/        # 硬件抽象层 (Hardware Abstraction Layer)
├── common/         # 通用库 (Utilities, Logging, RingBuffer)
└── bootloader/     # 引导程序 (可选)
```

- **规则**: 上层模块只能调用下层模块接口，禁止跨层反向调用或直接操作寄存器（必须通过 HAL）。

### 2. 技术栈与语言限制 (Tech Stack Constraints)

- **语言标准**: C (C99/C11) 或 Embedded C++ (C++14，禁用异常和 RTTI)。
- **编译器**: GCC (arm-none-eabi), ARMCC, IAR。
- **内存管理 (Memory Management)**:
  - ❌ **严禁动态内存**: 禁用 `malloc`, `free`, `new`, `delete`。所有内存必须在编译期静态分配，或在启动阶段从内存池分配一次。
  - ✅ **静态分配**: 使用全局数组、静态局部变量或固定大小的内存池 (Memory Pools)。
- **并发安全 (Concurrency)**:
  - 严禁在中断服务程序 (ISR) 中执行耗时操作或调用非重入函数。
  - 共享全局变量必须使用 `volatile` 修饰，并配合原子操作或临界区保护。

### 3. 命名规则 (Naming Conventions)

- **类型**: 必须使用 `<stdint.h>` 定义的定宽类型 (`uint8_t`, `int32_t`)。严禁使用 `int`, `short`, `long` 等依赖编译器的类型。
- **变量**:
  - 全局变量: `g_VariableName`
  - 静态变量: `s_VariableName`
  - 指针变量: `p_VariableName`
  - 局部变量: `variableName` (小驼峰)
- **函数**: `Module_Action` (e.g., `UART_Init`, `Sensor_ReadData`)。
- **宏/常量**: 全大写 `MAX_BUFFER_SIZE`。
- **枚举**: `EnumName_e` (类型), `ENUM_MEMBER_NAME` (值)。

### 4. 代码风格指南 (Style Guide) - 基于 MISRA C

- **大括号**: Allman 风格 (独占一行) 或 K&R，项目中必须统一。
- **注释**: Doxygen 格式，文件头必须包含 `@file`, `@author`, `@brief`, `@version`。
- **控制流**:
  - `switch` 语句必须包含 `default` 分支。
  - `if ... else if` 链必须以 `else` 结束。
  - 禁止使用 `goto` (除统一错误出口外)。
  - 禁止递归调用 (Recursion)，防止栈溢出。
- **防御性编程**:
  - 所有指针在使用前必须检查 `NULL`。
  - 所有数组访问必须检查边界。
  - 函数入口参数必须进行有效性检查 (`ASSERT`)。

### 5. Git 工作流规范 (Git Workflow)

- **分支命名**:
  - `feature/driver-uart`
  - `fix/isr-latency`
- **提交信息**: 遵循 Conventional Commits，必须包含模块前缀。
  - `feat(uart): add dma transmit support`
  - `fix(bsp): resolve gpio pin conflict`

### 6. 文档管理

- **硬件文档**: `docs/hardware/` (原理图, Datasheets)。
- **架构文档**: `docs/design/` (状态机图, 时序图)。
- **API 文档**: 代码中嵌入 Doxygen 注释，自动生成。
