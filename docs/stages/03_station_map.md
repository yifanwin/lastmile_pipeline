# 03 自动建立粗站位图

## 输入与门禁

必须有步骤 1 的有效人工批准及步骤 2 的 A 点物理通过。审批、协议、快照和旧稳定性轨迹哈希先核验；审批桌边限定候选范围。当前实现只针对 episode 14 / Cup_30，冻结稳定 grasp row 794。

## 与 MoMa-Kitchen 的对应

参考工程内 [数据生成流程解析](../../../MoMaKitchen/MoMa-Kitchen_数据生成流程解析.md)，采用“候选底盘位姿 → 独立操作试验 → 稀疏结果 → 局部 Gaussian 插值”的构建方式。该解析和本地仓库没有公开原始 collection loop，因此本管线是方法层对应，**不是复制其未公开代码或复现其机器人控制器**。

| 构建环节 | 本管线实现 |
|---|---|
| 目标附近采样 | 人工批准 north/south/west 的桌边操作带；沿边约 0.30 m，距桌边 0.32/0.48/0.64 m，朝向目标 ±20° 和正对目标；包含精确 A 点 |
| Reach 半径限制 | 使用 RBY1 双臂/合法联动躯干的保守手指接触上界，不将其称为论文机械臂最大 reach；界外点保留为解析诊断 |
| 操作 oracle | MuJoCo 恢复同一全状态，再只设置待测机器人底盘；cuRobo 双臂 × 五个 h 筛选三段路径；关键候选真实执行严格 Pick |
| 稀疏标签 | 只有完整动态 Pick 原始轨迹通过/失败才记 1/0；几何过滤、规划无解或未完成 Pick 不强行二值化 |
| Gaussian interpolation | 同一 XY 的已测配置标签先求均值，再做 kNN 高斯插值（不是重复随机试验成功率），k=3、σ=0.12 m，离实测点超过 0.30 m 保持 NaN；不把未知地面填成失败 |
| 训练 episode / RGB-D | 本阶段不生成；用户当前只需要粗站位图 |

## 执行

```bash
lastmile_pipeline/bin/lastmile station-map --case-id case1-cup-001 --run-id coarse_v1
# 被中断后：配置、审批及场景不变时允许续跑
lastmile_pipeline/bin/lastmile station-map --case-id case1-cup-001 --run-id coarse_v1 --resume
# 本地只读站位图服务（默认 18766）
lastmile_pipeline/bin/lastmile serve-station-map --case-id case1-cup-001 --run-id coarse_v1
# 从结果重画（不进行新规划或物理执行）
lastmile_pipeline/bin/lastmile plot-station-map --case-id case1-cup-001 --run-id coarse_v1
```

默认 CUDA 5。新配置须使用新的 run_id；不得覆盖旧证据。续跑检查冻结配置、审批和审阅包哈希，逐 pose/arm/h 保留检查点。若物理试验被中断，没有最终结果的 trial 会重新从原始快照开始，不承接被推歪的目标。

## 几何、规划、物理严格分层

- 地面覆盖来自真实 floor 几何；初始碰撞来自 MuJoCo 完整机器人模型。它们只是筛选，不是导航通过。
- 解析接触上界独立于 IK 搜索；初始碰撞、界外和预算无解分别保留原因。有限预算无解不等于双臂/连续躯干不可达。
- 规划保留全部五个 h、双臂及冻结抓法；完整路径指预抓取、接近和抬升均规划成功。不能当作真实成功。
- 物理执行先恢复源快照，然后仅放置底盘。保持原始物体、目标、头部和闲置臂状态，动态调整躯干并重新规划，不使用诊断中直接设置的 h 作为执行起始状态。
- 放置底盘只是独立操作标注初始化，**不代表曾导航到该点**。每个 trial 保存放置记录、放置快照、躯干轨迹、Pick 轨迹和严格验收。
- XY 图合并同一位置的采样朝向：任一采样 yaw 有真实成功就标绿；完整 pose/yaw 信息保留 CSV/JSON。各站位初步单次试验不估计成功率。

## 输出

`cases/<case-id>/03_station_map/runs/<run-id>/`：

- `frozen_inputs.json`：冻结配置、来源/审批/协议哈希与事先预期。
- `candidates.json`、`reach_bound.json`、`floor_grid.npz`：采样和几何过滤。
- `planning.json`：逐 pose × arm × h 的 IK/路径结果。
- `physics_selection.json`、`physics.json`、`trials/`：先 A 点控制，再空间分散的路径通过点和边界诊断；最多 12 次。
- `stations.json`、`station_table.csv`：完整含 yaw 的站位明细。
- `station_map.png / .svg / .html`：可交付站位图、自包含阅读页。
- `floor_sparse_samples.csv`：真实二值试验到最近地面点的对应，不把空白填成 0。
- `evidence_audit.json`、`evidence_manifest.json`：原轨迹重新验收、逐 trial 非 base 状态一致性及哈希清单。
- `gaussian_preview.npz`：局部插值及邻居数，未知为 NaN。
- `summary.json`：覆盖、物理结果及适用范围。

完成本阶段不等于构造完成。后续步骤 4 由人工选构造方式；步骤 5 仍必须验证真实绕行与到站抓取。
