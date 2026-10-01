# Last-mile 数据生成工作区

这是按 [构建方案](../lastmile%20case%20构建方案.md) 新建的独立工作区。原 benchmark、资产、`molmospaces` 和旧 worktree 保持不变。结果以 [校准报告](reports/REPORT.md) 为准，未通过物理验收的资产不能进白名单。

## 目录

```text
lastmile_pipeline/
├── configs/             # 动作协议、工程阈值、有限搜索预算、候选索引
├── src/lastmile/         # 动作桥、规划配置、只读恢复、严格 Pick、准入判定
├── scripts/             # 审计、模型校准、GPU 检查、正例复测入口
├── tests/               # 单元测试（含拒绝伪阳性与把手回退）
├── registry/
│   ├── provenance.json  # benchmark/方案哈希
│   ├── episodes/        # 派生恢复配置，含用户授权的替代场景，不继续使用 PnP success
│   ├── assets/          # 7 种资产的源文件、grasp row ID、审核状态
│   ├── planner_configs/ # 左右臂 × 5 个 h 的单臂配置
│   └── whitelist.json   # 正式资产 + 合规 grasp 白名单；默认空
├── reports/             # 小型结果表、报告、诊断日志
└── runs/                # 大型物理轨迹、完整快照；不入版本管理
```

## 复现

不安装或同步任何依赖，复用仓库现有 uv 环境：

```bash
bash lastmile_pipeline/scripts/run_offline.sh
source molmospaces/setup_env.sh
export PYTHONPATH="$PWD/lastmile_pipeline/src:$PYTHONPATH"
# 必须放在 source 之后，原脚本默认 GPU 3；选物理卡 5 时 torch 内索引为 0。
export CUDA_VISIBLE_DEVICES=5 MUJOCO_EGL_DEVICE_ID=5 MUJOCO_GL=disable
molmospaces/.venv/bin/python lastmile_pipeline/scripts/probe_gpu.py
molmospaces/.venv/bin/python lastmile_pipeline/scripts/run_calibration.py --index 1949
molmospaces/.venv/bin/python lastmile_pipeline/scripts/run_calibration.py --index 14
```

GPU 命令需要宿主设备访问权限；沙箱内 CUDA 检测可能返回 False。`MUJOCO_GL=disable` 只关闭渲染，不关闭碰撞和动力学。场景文件位于 NAS，首次解析/编译会慢。

## A/B 验收边界

- 协议：20 维，底盘增量 3 + 双臂各 7 + 双夹爪各 1 + 绝对躯干参数 1；头部固定。
- 单臂 cuRobo：局部底盘系中 base 锁零；躯干、闲置臂、头、夹爪同步状态；规划器实际优化关节必须恰好 7 个。
- `h` 是角参数，不是米。连续调整需动态检查；离散高度失败不证明连续范围无解。
- 搜索：每个手臂/高度最多 24 个候选，所有候选恢复同一快照。grasp ID 是源 NPZ 的行号。float16 旋转做确定性 SO(3) 投影，不能重新编号。
- 固定底盘操作图必须基于物理记录，不能把 IK 或 `success=True` 当通过。
- 正式准入：名义 5/5，小扰动至少 19/20；每次独立恢复。未执行的记录不进入分母。
- 把手：需要人工确认局部部件区域与实际双指接触；缺少标注时拒绝启动把手模式，不回退到杯身/杯沿。
- P-01 未复现且原因未定位时，不扩展站位搜索；不开展阶段 C/D。

校准器支持 `--world-geometry mesh`（真实 MuJoCo 碰撞 geom）与 `aabb`（保守根体近似）；默认 AABB 仅用于诊断，不据其失败断言物理不可达。球/圆柱等图元在规划 mesh 中离散化，仿真碰撞始终保持原模型。把手执行器通过确认标注后才启用；目前区域与合规 grasp 尚未人工确认，真实把手验收未通过。物理门槛是统一待校准工程值，不是已验证的物理规律。

## 修正后的物理复测

```bash
# 首次运行不带 --cache-dir；之后复用同一 source_index 的已编译模型。
molmospaces/.venv/bin/python lastmile_pipeline/scripts/run_calibration.py \
  --index 14 --arms left --heights .738 --run-id MY-CUP-MESH \
  --world-geometry mesh --time-dilation .25 --approach-overshoot .01 \
  --cache-dir lastmile_pipeline/runs/stage_a/E002-corrected/0014_seed0
```

`repeat_nominal.py` 从完整场景初始快照重新执行躯干调整和冻结控制序列，逐物理步重新测量、重新判定；不复制原 verdict。它验证确定性控制器的名义复现，不能当作随机规划重复率。`repeat_perturbed.py` 预注册 20 个 ±1 cm / ±2° 到站误差、规划种子及单个 grasp row，然后逐次完整恢复、重规划、真实执行，不回退其他抓法。两类证据的准入协议哈希必须一致，不能直接拼接不同试验设计的结果。

`replay_trace.py` 用于定位轨迹恢复误差，不产生新成功声明。`Simulation.sample` 的 force-aware-v2 采样器检查实际接触法向力，正距离软接触也可构成支撑；仅有 penetration 不能认定双指承力。旧版距离采样结果需力感知复核后才考虑准入。
