# OpenHumanoid Motion Compiler 阶段性开发与验证报告

**报告日期：** 2026-08-18

**项目版本：** `0.1.0.dev0`

**证据基线提交：** `9f8b73a235bbbd7dc0a84648c78cc47647e0b2a2`

**项目地址：** <https://github.com/Ethan-Schenberg/openhumanoid-motion-compiler>

**开源许可：** Apache-2.0

**项目阶段：** 早期原型（Pre-Alpha）

## 1. 执行摘要

OpenHumanoid Motion Compiler（OHMC）是一个面向人形机器人的开源动作编译与基准验证工具。项目将人体动作转换视为一个“编译器”问题：先将 BVH 等来源解析为机器人无关的标准动作表示，再经过形态与时间归一化、语义映射、约束逆运动学和轨迹质量检查，最后为不同厂商的人形机器人生成可复核的仿真证据包。

本阶段已建立一条可运行、可测试、可审计的离线管线，并将 Unitree G1 与 AgiBot X2 Ultra 作为一等支持目标。最终本地验证结果为 59 项测试全部通过；GitHub Actions 在 Python 3.10、3.11 和 3.12 上全部通过；简单动作矩阵 4/4 通过，多肢体矩阵 2/2 通过；独立证据审计共校验 75 个产物，未发现完整性问题。

本报告中的“通过”仅表示指定的离线数据合同、运动学求解、质量检查、MuJoCo `mj_forward` 回放和证据完整性条件成立，不代表已经通过控制器动力学、真机试验或安全认证。

## 2. 问题与项目价值

人形机器人动作演示往往将动作来源、骨架约定、逆运动学求解器、机器人模型与厂商接口紧密耦合。这类实现虽然能够快速生成演示，却很难回答以下问题：

- 输入动作的授权、坐标系、单位和骨架定义是什么？
- 轨迹经历了哪些转换，每一步的输入与输出是否可验证？
- 同一动作如何不经手工改文件就适配不同机器人？
- 失败帧、关节限位、动态限制缺失和映射不完整是否被真实记录？
- 外部审阅者能否验证产物没有被替换、遗漏或静默修改？

OHMC 的核心价值不是产出某一支特定舞蹈，而是把动作处理转换成一个具有显式语义、版本化数据合同、确定性转换、严格失败状态和可独立审计证据的工程过程。

## 3. 技术架构

```text
已授权的 BVH 动作
        |
        v
BVH 解析与坐标系/单位显式化
        |
        v
Canonical Motion（局部四元数 + 世界位姿）
        |
        v
形态缩放 + 时间重采样 + FK 重算
        |
        v
语义任务映射 + 约束 IK
        |
        v
Motion IR + 轨迹质量报告
        |
        +------> Unitree G1 适配合同
        |
        +------> AgiBot X2 Ultra 适配合同
        |
        v
MuJoCo 运动学回放 + 原子证据包 + 独立完整性审计
```

架构将共享核心与厂商细节分离：动作语义、IK 问题和证据格式使用厂商无关的 JSON Schema；关节顺序、可控白名单、限位、模型解析方式和消息顺序则由声明式 profile 与 adapter 负责。这使得添加机器人目标时无需重写整条管线。

## 4. 已完成的核心能力

### 4.1 标准动作与可追溯转换

- 按 BVH 声明顺序处理位移与旋转通道。
- 要求调用者显式给出源坐标系、长度单位和 SPDX 许可，不自动猜测。
- 输出局部四元数、局部平移和通过前向运动学重建的世界位姿。
- 支持形态比例缩放与确定性重采样；旋转插值使用最短弧四元数 SLERP，并保留原动作精确结束时刻。
- 每个编译 pass 记录输入哈希、输出哈希与配置哈希，形成可追溯链。

### 4.2 求解器无关的约束 IK

- 建立 IK task map、problem 和 result 三类版本化数据合同。
- 实现确定性、有界的阻尼最小二乘 MuJoCo 参考求解器。
- 逐帧记录是否求解、残差、迭代状态与活跃限位。
- 不使用“上一帧替代失败帧”等静默回退；存在失败帧时拒绝将结果编译为有效 Motion IR。
- 验证 robot profile 与模型关节限位的一致性，并将模型哈希锁定到结果。

### 4.3 双厂商目标支持

OHMC 将两家厂商的软件、模型和接口作为主仓库能力管理，而非留给非官方下游分支：

| 目标 | 已纳入的能力 | 当前证据边界 |
|---|---|---|
| Unitree G1 | 29 DoF profile、官方模型解析、语义映射、多肢体 IK、`LowCmd` 顺序合同、SDK/ROS 2/MuJoCo 锁定依赖 | 已完成离线模型与接口合同验证；尚未在 CI 中编译真实 SDK 适配器 |
| AgiBot X2 Ultra | 30 个可用命令变量的 profile、官方 X2 URDF 解析、语义映射、多肢体 IK、`JointCommandArray` 顺序合同、AimDK 与 URDF 校验流程 | 已完成离线模型与接口合同验证；AimDK 再分发许可不完整，因此采用本地导入和 SHA-256 校验，不将上游二进制包复制进 Git |

此处的“支持”指已纳入 profile、官方模型解析、语义映射、仿真目标、接口 fixture 和回归测试；不表示已开放真机传输。

### 4.4 一键双厂商矩阵

`ohmc simulate INPUT.bvh --target all` 会根据输入动作合同选择兼容的目标族，将每个目标隔离执行，保留单目标失败，并原子化生成矩阵清单。当前包含：

- `simple_motion_v1`：4 个官方/合同目标。
- `full_body_landmarks_v1`：2 个 G1/X2 多肢体基准目标。
- 每个子目标拥有独立 manifest，根矩阵记录其相对路径、状态和清单哈希。
- 一个目标的失败不会删除其证据，也不会被其他目标的成功覆盖。

### 4.5 轨迹质量与独立证据审计

轨迹质量报告已覆盖关节位置范围、速度、加速度、加加速度（jerk）和最小位置余量，并记录对峰值负责的关节。对于 profile 未声明的动态限制，系统生成 warning，而不将“没有数据”冒充为“已通过”。

`ohmc verify-evidence` 可在不重新运行编译器的情况下独立检查：

- bundle 或 matrix manifest 是否符合 schema；
- 产物路径是否为安全相对路径；
- 已声明的每个产物是否存在且 SHA-256 一致；
- 矩阵中的子 bundle 清单哈希是否一致；
- 执行结果失败与证据本身的完整性是否被分开表达。

## 5. 验证结果

### 5.1 自动化测试与持续集成

| 检查项 | 结果 |
|---|---:|
| 本地 pytest | 59/59 通过 |
| Python 源码编译检查 | 通过 |
| JSON Schema Draft 2020-12 自验证 | 通过 |
| GitHub Actions / Python 3.10 | 通过 |
| GitHub Actions / Python 3.11 | 通过 |
| GitHub Actions / Python 3.12 | 通过 |

对应 CI 运行：<https://github.com/Ethan-Schenberg/openhumanoid-motion-compiler/actions/runs/32137675374>

### 5.2 发布矩阵与证据完整性

| 矩阵 | 输入合同 | 目标结果 | 已校验 bundle | 已校验产物 | 完整性问题 |
|---|---|---:|---:|---:|---:|
| 简单动作矩阵 | `simple_motion_v1` | 4/4 通过 | 4 | 49 | 0 |
| 多肢体矩阵 | `full_body_landmarks_v1` | 2/2 通过 | 2 | 26 | 0 |
| **合计** | — | **6/6 通过** | **6** | **75** | **0** |

两份审计报告均记录 `hardware_commands_sent: false`。

### 5.3 多肢体 IK 基准

基准使用项目自建的 CC0 全身 BVH fixture，共包含 16 个必需标志点、51 个动作通道、3 帧，采样率为 50 Hz。

| 目标 | 可用命令变量 | 求解帧 | 失败帧 | 峰值位置残差 | 源标志点覆盖 | IK 任务标志点覆盖 |
|---|---:|---:|---:|---:|---:|---:|
| Unitree G1 | 29/29 | 3 | 0 | 0.004166 m | 16/16 | 9/16 |
| AgiBot X2 Ultra | 30/30 | 3 | 0 | 0.004188 m | 16/16 | 9/16 |

两个目标的峰值残差均低于当前 5 mm 基准门限。但是，当前仅使用 Chest、双侧 Elbow/Wrist 与双侧 Knee/Ankle 等 9 个位置任务；Hips、Spine、Head、双 Shoulder 和双 Hip 尚未形成独立 IK 任务。因此，当前结果被准确标注为“约束部分身体 IK”，而不是完整的全身 IK。

## 6. 可复现性与开源工程质量

项目已配置以下开源协作基础：

- Apache-2.0 许可证、贡献指南、行为准则、安全政策和维护者文件。
- 可编辑安装的 Python 包与 `ohmc` 命令行入口。
- 14 类版本化 JSON Schema，覆盖标准动作、Motion IR、机器人 profile、IK、质量、仿真 bundle、目标矩阵与证据审计。
- 固定版本与哈希的厂商依赖清单，同时保留上游许可和再分发边界。
- 可重复执行的合成 fixture 与严格的失败测试。
- 按输入、模型、配置和产物 SHA-256 组织的证据链。

当前仓库在 `src/` 与 `tests/` 中共有约 6,523 行 Python 代码（包含测试）。代码量仅作为开发规模参考，不作为质量指标；项目质量主要由数据合同、自动测试、可复现矩阵和独立审计证明。

## 7. 安全、许可与声明边界

### 7.1 真实机器人

当前版本不包含默认可用的真机命令传输。所有发布矩阵、子 bundle、回放报告与审计报告均将是否发送硬件命令作为机器可读字段。本次验证值为 `false`。

URDF 和 MuJoCo 模型仅能证明模型结构与离线运动学行为，不构成真机控制权限、控制频率、实时接口或安全性的证明。

### 7.2 厂商依赖与再分发

- Unitree 公开仓库使用锁定提交，依照其开源许可保留声明。
- AgiBot X2 URDF 与 AimDK 使用官方来源与 SHA-256 校验。
- 对于未找到完整再分发许可的厂商产物，OHMC 只提供获取/导入、校验和适配路径，不直接复制到项目发布物中。

### 7.3 当前未完成的事项

为避免把原型结果过度包装，以下能力不在本阶段已完成范围内：

- 16/16 全标志点 IK 任务覆盖，以及端部方向约束。
- 足底接触保持、自碰撞检查、动力学平衡与控制器闭环仿真。
- 基于执行器的力矩、温度、电流或热限制验证。
- Unitree SDK2/ROS 2 与 AgiBot AimDK 真实适配器的可编译 CI 作业。
- 真实机器人上的执行验证与安全认证。
- 侧视对比 MP4、图表和紧凑 HTML 基准报告。

## 8. 风险评估

| 风险 | 影响 | 当前控制 | 下一步 |
|---|---|---|---|
| 位置任务覆盖不等于完整姿态保真 | 头部、躯干和肩髋姿态可能丢失 | 报告明确标记 9/16 与 partial-body | 增加方向任务、root/躯干姿态与多目标权重标定 |
| 运动学回放不代表可控动力学 | 轨迹可能在真实执行器上不可跟踪 | 所有 manifest 明确关闭 dynamics/hardware 声明 | 加入执行器模型、位置控制器与回归门限 |
| 厂商接口可能变更 | 适配器与上游不兼容 | 锁定 commit/checksum，保留接口 fixture | 建立真实 SDK 编译矩阵与版本升级检查清单 |
| 短基准难以暴露长时稳定性问题 | 尖峰、累积漂移或接触问题可能被遗漏 | 保留速度/加速度/jerk 指标和失败样例 | 引入多时长、多形态和反例动作集 |
| 依赖许可不完整 | 不能合法重新分发某些二进制产物 | 本地导入、校验、Git 排除 | 完善 SBOM/NOTICE 并向上游获取明确授权 |

## 9. 下一阶段建议目标

建议按照“数学完整性 → 动力学可验证性 → 视觉展示 → 真实 SDK 合同”的顺序继续开发：

1. **全身 IK 合同升级：** 将任务覆盖从 9/16 扩展到 16/16，增加端部方向、root/躯干姿态、权重和容差的标准化定义。
2. **接触感知的闭环 MuJoCo 回放：** 引入执行器和控制器，测量足滑、基座漂移、接触不连续与力矩代理指标。
3. **可视化基准证据：** 从同一 manifest 生成源骨架、G1 和 X2 并排 MP4，以及包含误差和限制曲线的静态 HTML 报告。
4. **真实厂商开发环境合同：** 在隔离容器或厂商兼容主机中编译 Unitree SDK2/ROS 2 和 AgiBot AimDK 适配器，使用录制消息完成单位、字段顺序、时戳和 QoS 校验。
5. **第三方可复现发布：** 建立干净 Ubuntu 安装测试、离线依赖模式、签名发布物、SBOM 和基准数据迁移政策。

## 10. 结论

OHMC 已经从概念文档发展为一个可执行的双厂商动作编译原型。它的主要工程成果是建立了从有授权的动作输入到标准表示、约束 IK、双厂商轨迹、离线回放和可独立审计证据的端到端链路。

当前证据足以支持“可复现、可审计、跨 Unitree G1 与 AgiBot X2 Ultra 的离线运动学原型”这一结论；尚不足以支持“完整全身动力学控制”或“可安全下发真机”的声明。下一阶段的价值在于通过完整姿态任务、接触动力学、真实 SDK 编译和第三方复现进一步提高证据层级。

## 附录 A：关键证据索引

- 简单矩阵审计：`build/release-simple-matrix-v1-audit.json`
- 多肢体矩阵审计：`build/release-full-matrix-v1-audit.json`
- 简单矩阵清单：`build/release-simple-matrix-v1/matrix-manifest.json`
- 多肢体矩阵清单：`build/release-full-matrix-v1/matrix-manifest.json`
- IK 数据合同：[`IK_CONTRACT.md`](IK_CONTRACT.md)
- 标志点覆盖合同：[`LANDMARK_COVERAGE.md`](LANDMARK_COVERAGE.md)
- 目标矩阵合同：[`TARGET_MATRIX.md`](TARGET_MATRIX.md)
- 证据审计合同：[`EVIDENCE_AUDIT.md`](EVIDENCE_AUDIT.md)
- 轨迹质量门禁：[`QUALITY_GATES.md`](QUALITY_GATES.md)
- 项目路线图：[`ROADMAP.md`](ROADMAP.md)

## 附录 B：复核命令

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e '.[dev,mujoco]'
.venv/bin/python -m pytest -q

.venv/bin/ohmc simulate examples/simple_motion.bvh \
  --target all \
  --source-license CC0-1.0 \
  --source-convention right_handed_x_right_y_up_z_backward \
  --source-length-unit m \
  --output build/review-simple \
  --cache-dir .ohmc-cache
.venv/bin/ohmc verify-evidence build/review-simple \
  --report build/review-simple-audit.json

.venv/bin/ohmc simulate examples/full_body_motion.bvh \
  --target all \
  --source-license CC0-1.0 \
  --source-convention right_handed_x_forward_y_left_z_up \
  --source-length-unit m \
  --output build/review-full-body \
  --cache-dir .ohmc-cache
.venv/bin/ohmc verify-evidence build/review-full-body \
  --report build/review-full-body-audit.json
```

上述命令只执行本地编译、验证和仿真证据生成，不会向真实机器人发送命令。
