# OHMC 2.0 X2 训练平台

状态：基础平台已实现，目标 WSL2/GPU 与真机验证待执行

OHMC 2.0 为 AgiBot X2 Ultra T2.5 增加了一个本地、分阶段、默认无真机权限的强化学习工作流。它不会把“仿真里能动”显示成“真机已验证”。

## 小白先看这里

在 WSL2 中运行：

```bash
ohmc doctor
ohmc web
```

然后在 Windows 浏览器打开 <http://127.0.0.1:8000>，选择“自主步态训练”。

“创建、预检并启动”只做训练阶段：

1. 检查 WSL2、NVIDIA GPU、Isaac Lab 版本、RSL-RL 版本、X2 URDF 哈希和网格文件。
2. 创建不可覆盖的运行目录、SQLite 任务记录和 `RunManifest`。
3. 按站立、平地、坡地、不平地、低障碍、楼梯顺序训练。
4. 导出 checkpoint、ONNX 和 10 秒动作预览视频。
5. 停在“等待评估”，不会连接 X2，也不会自动进入真机阶段。

网页会显示诊断项、修复建议、训练日志、事件、指标、文件哈希和动作视频。暂停、继续、取消会作用于整个训练进程组。WSL2 异常退出后，再次打开网页会把失联任务标记为可恢复；“从课程检查点恢复”只续接该任务自己的 checkpoint。

## 两条通道

| 通道 | 用途 | 边界 |
|---|---|---|
| LinkCraft 快速表演 | 视频/BVH 生成厂商动作资源 | 独立官方平台；OHMC 只给入口，不代替官方控制 |
| OHMC 自主训练 | 从零步态、PPO、证据与策略包 | 仿真研究；任何真机阶段都需另行人工批准 |

LinkCraft 官方入口：<https://linkcraft.agibot.com/>

## 状态机

```text
预检 → 训练 → 评估 → Sim2Sim → 等待人工审核 → 真机候选
  └────────────── 任一失败停在当前阶段 ──────────────┘
```

- `训练完成`：Isaac Lab 后端正常退出并生成 checkpoint、ONNX 和预览；尚未评估。
- `仿真通过`：控制器仿真、运行时故障注入和独立 Sim2Sim 的所有门禁都通过。
- `真机候选`：只表示 Policy Bundle 完整且经人工审核；仍没有硬件执行权限。
- `真机已验证`：只有操作者完成对应硬件阶段并补充证据后才能使用。本版本不会自动写出这个标签。

## 公共命令

```bash
# 只读环境检查
ohmc doctor

# 创建并运行一个新任务
ohmc train examples/training/x2_rgbd_rough_ppo_v1.yaml

# 断电后仅恢复原任务自己的课程 checkpoint
ohmc train --resume-run RUN_ID

# 导入至少 100 回合的控制器、MuJoCo/官方模拟器和故障注入指标
ohmc evaluate RUN_ID --metrics evaluation-metrics.json

# E3 Sim2Sim 通过后制作或复核仿真策略包
ohmc deploy prepare RUN_ID --policy policy.onnx --output policy-bundle
ohmc deploy verify policy-bundle

# 本地网页
ohmc web
```

所有命令默认使用 `build/training-runs`。`deploy prepare` 只复制、哈希和审计文件，不安装软件到机器人，也不发送关节指令。

## 固定训练契约

- X2 策略动作顺序固定为 29 关节，头部两个关节完全排除。
- 策略输出是缩放为 `0.25` 的残差关节位置，策略频率 50 Hz；500 Hz 插值属于后续厂商部署运行时。
- Actor 输入只包含真机可获得的角速度、重力方向、速度命令、29 关节位置/速度、上一动作、RGB、深度和图像年龄。
- Critic 额外读取基座线速度、接触力和地形高度，形成非对称 Actor-Critic。
- RGB 与深度均为 `64×48 @ 15 Hz`；两个独立 CNN 通过全局池化各输出 32 维特征。
- 视觉训练包含 RGB 增益、曝光、低频纹理、噪声，以及深度噪声、空洞、RGB-D 错位和 0–2 个策略周期延迟。
- 生产配方只能随机初始化。课程阶段续接同一运行自己的 checkpoint，不等同于导入预训练模型。

这些定义写入 `TrainingRecipe` 和最终 `PolicyBundle`，不是只存在于代码注释中。

## 目标环境安装

根项目环境负责网页、任务队列和证据：

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e '.[dev,mujoco,web]'
.venv/bin/ohmc vendor sync agibot-x2
```

Isaac Lab 必须使用单独的固定 Python 3.12 运行时。完整步骤见 [`integrations/isaaclab/README.md`](../integrations/isaaclab/README.md)。完成后设置：

```bash
export OHMC_ISAACLAB_ROOT=/workspace/IsaacLab
export OHMC_ISAACLAB_PYTHON=/workspace/IsaacLab/_isaac_sim/python.sh
export OHMC_X2_URDF=/workspace/ohmc/.ohmc-cache/sources/agibot_x2/urdf/X2_URDF-v1.3.0/x2_ultra.urdf
```

RTX 3070 8 GB 默认 256 个并行环境。只有 `nvidia-smi` 显示的显存达到配方门槛时，医生才建议 512；用户不能通过网页跳过这个检查。

## 当前已经证明与尚未证明

已由本地自动化测试覆盖：

- 五类版本化 JSON Schema、严格状态转换、SQLite 持久化与原子清单；
- 29 关节顺序、头部排除、随机初始化和 Actor/Critic 观察边界；
- 评估门禁、Policy Bundle 哈希及篡改检测；
- 本地网页无硬件/关节控制路由，目录穿越被拒绝；
- 中断恢复、固定 Isaac/RSL 版本和六阶段课程的静态契约。

当前开发机不是 WSL2 RTX 主机，因此尚未证明：

- X2 URDF 在固定 Isaac Lab 版本中完成站立初始化；
- RTX 3070 上 256 环境的显存占用、吞吐和断电恢复；
- 任何一轮真实 PPO 训练能达到验收阈值；
- ONNX 在独立 MuJoCo、官方 `x2_rl_deploy` 模拟器或 X2 真机上通过。

这些缺口会被 `ohmc doctor` 或评估状态机挡住，不能通过手工改网页标签绕过。

## 真机安全边界

本仓库没有网页到关节命令的接口，也不把训练软件安装到 PC1 运动控制机。PS5 持续按住启用、松开/断连/图像超时进入阻尼，以及 100 ms 撤权，需要在厂商允许的部署主机和官方运行时中实现并做故障注入。通过前，Policy Bundle 的 `hardware_execution` 永远是 `false`。
