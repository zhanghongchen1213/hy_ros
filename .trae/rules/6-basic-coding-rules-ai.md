# 基础编程规则 (Basic Programming Rules) - 场景C: AI 深度学习

## 🎯 核心目标
本文件定义了 **AI 深度学习开发场景**下的项目“宪法”。确立基于 **PyTorch Lightning Style Guide** 和 **Google Python Style for ML** 的编码规范，确保实验可复现、模型可维护、代码可扩展。

## 💡 核心理念
**可复现性，模块化实验 (Reproducibility, Modular Experimentation)**。
AI 开发不仅仅是写脚本，而是构建系统。我们优先选择**配置化**、**解耦模型与数据**、**版本化实验**。

## 📝 规则详情

### 1. 项目结构规范 (Project Structure)
采用 PyTorch Lightning 推荐结构（Hydra 模板变体）：
```text
my_ai_project/
├── configs/            # 配置文件 (Hydra .yaml)
│   ├── model/          # 模型参数
│   ├── data/           # 数据集参数
│   └── experiment/     # 实验运行配置
├── data/               # 数据集 (不提交到 git)
├── logs/               # 日志与 Checkpoints (不提交到 git)
├── src/
│   ├── models/         # LightningModules
│   ├── datamodules/    # LightningDataModules
│   ├── utils/          # 工具函数
│   └── train.py        # 训练入口
├── notebooks/          # 探索性 Notebooks
├── requirements.txt    # 依赖列表
└── README.md           # 实验说明
```
- **规则**: 严禁在 `src` 中硬编码路径或超参数，所有可变参数必须通过 `configs/` 注入。

### 2. 技术栈与语言限制 (Tech Stack Constraints)
- **框架**: PyTorch (2.0+), PyTorch Lightning (强制使用，规范化训练循环)。
- **配置管理**: Hydra (管理复杂的实验配置)。
- **语言标准**: Python 3.9+ (强制使用 Type Hints)。
- **加速库**: CUDA, cuDNN (版本必须锁定)。
- **显存管理**:
    - 严禁在训练循环中保留不必要的计算图引用（如 `loss.item()` 而非 `loss`）。
    - 必须在 `validation_step` 结束后清理缓存。

### 3. 命名规则 (Naming Conventions)
- **类名**: `PascalCase` (e.g., `ResNetEncoder`, `Cifar10DataModule`)。
- **函数/变量**: `snake_case` (e.g., `train_step`, `learning_rate`)。
- **张量变量**: 建议添加形状注释 (e.g., `batch_images  # [B, C, H, W]`)。
- **私有成员**: `_private_method`。

### 4. 代码风格指南 (Style Guide)
- **Type Hints**: 所有函数签名必须包含类型注解。
    ```python
    def forward(self, x: torch.Tensor) -> torch.Tensor: ...
    ```
- **Docstrings**: 使用 Google Style，必须说明 `Args` (包含 Tensor Shape) 和 `Returns`。
- **Notebooks**: 仅用于探索 (Exploration)，严禁将生产代码留在 Notebook 中。所有核心逻辑必须移至 `src/`。
- **随机种子**: 必须使用 `seed_everything` 固定所有随机源 (NumPy, PyTorch, Python)。

### 5. 实验管理规范 (Experiment Management)
- **日志**: 使用 `WandB` 或 `TensorBoard` 记录 Metrics。
- **Checkpoints**:
    - 必须保存 `best_model.ckpt` (基于 val_loss)。
    - 必须保存 `last.ckpt` (用于断点续训)。
- **版本控制**:
    - 每次实验必须记录 git commit hash。
    - 严禁在 dirty git state 下运行正式实验。

### 6. 文档管理
- **README.md**: 必须包含环境复现步骤 (`conda env create...`) 和 训练/推理的 CLI 命令示例。
- **Results**: 实验结果汇总表（最好链接到 WandB Project）。
