# 步骤 5：A* 连续导航与到站抓取

已实现 Case 1-R 的 `robot_only` 困难起点验收。

## 输入、门禁与执行

先检查步骤 1–3 和当前构造证据哈希。当前输入为步骤 3 的 P131 超范围候选，执行时从其派生初始快照恢复；目标和家具保持源初始状态。

```bash
lastmile_pipeline/bin/lastmile closed-loop --construction-run-id robot_move_v3 --run-id astar_v2 --trials 3
MUJOCO_GL=disable lastmile_pipeline/bin/lastmile audit-closed-loop --run-id astar_v2
lastmile_pipeline/bin/lastmile render-closed-loop --run-id astar_v2
```

以上当前运行不应覆盖；新实验需使用新 run-id。默认 CUDA 5，亦可 `CUDA_VISIBLE_DEVICES=0`。完成的运行目录禁止覆盖；失败轨迹原样保留，停止该次重复序列。

执行：A* 到安全停靠点 → 实际底盘转向 → 最后一段 A* 接近 A → 保留完整实测到站状态 → 动态调整躯干 → cuRobo 重新规划预抓取、接近、闭合、抬升、保持。

每个几何拐点减速停车再换向，线加速度 .005 m/s²、最大速度 .06 m/s，角加速度 .005 rad/s²、最大角速度 .025 rad/s。约束不放宽；导航全过程双臂与头固定、躯干 h=0，禁止机器人环境承力接触（底盘地面接触除外）。目标位移不得超过 2 mm，旋转不得超过 .01 rad。

## 证据与成功条件

- 起点失败是双臂、全 yaw、连续合法躯干范围的保守接触界证书，**不伪造固定底盘执行失败动作**，也不声称整条西侧不可操作。
- 导航状态逐 .004 s 记录；到站误差必须在 2 mm / .2° 内。
- 保存 `actual_arrival_snapshot.npz`，其 base 与抓取调整前缀初始 base 必须完全一致；下一物理 tick 时间间隔 .004 s。进入导航后禁用恢复源快照。
- 抓取沿用既有 strict Pick：双指真实承力、无支撑、至少抬升 .05 m、稳定保持至少 2 s，以及底盘/躯干/头/闲置臂约束。
- 默认三次新连续试验，任何一次失败即停止、保存原始证据。三次全通过才标记重复验收完成，不自动完成人工最终接受。

视频仅回放成功试验保存的真实 qpos，不生成或插值运动。导航 8×、躯干调整 4×、抓取 1×，逐段标注；墙体仅渲染透明，仿真碰撞始终保留。交付第二张只读证据审阅卡；步骤 6 的最终接受/数据集导出仍需独立实现和人工介入。
