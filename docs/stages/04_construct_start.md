# 步骤 4：人工方式选择与原样筛选

已实现 `original_only` 分支，其他修改方式未实现。对应方案第六节优先级 1：原样筛选。

## 输入与执行

- 步骤 1 有效人工批准；步骤 2 严格抓取通过；步骤 3 原始证据审计及文件哈希通过。
- `04_construct_start/human_choice.json` 记录人工选定的方式。当前来自用户明确回复“仅筛选原始起点，不改场景”，不是 agent 批准候选。
- `configs/04_construct_start.yaml` 禁止改变底盘初始位姿、目标、家具和资产。

```bash
MUJOCO_GL=disable lastmile_pipeline/bin/lastmile construct-original --case-id case1-cup-001
MUJOCO_GL=disable lastmile_pipeline/bin/lastmile check-construct-gate --case-id case1-cup-001
```

第一条用于首次筛选，已有结果时拒绝覆盖。当前结果已经生成，不必重跑。第二条当前应退出 1，说明步骤 5 不放行，而非运行故障。

## 判定与输出

同一原始底盘位姿存在任何符合协议的抓取成功证据，即筛除为 Case 1 失败起点候选。位姿匹配容差为平移 1e-5 m、朝向 1e-5 rad；允许协议内执行躯干调整，不修改源快照。重新检查躯干调整前缀与完整 Pick 原始轨迹，而非只读取旧成功字段。

没有成功证据、搜索耗尽或单次执行失败，都不能自动判为纯距离不可达。此时保留待查状态，仍不放行。

输出：`summary.json`、`screening_registry.json`、`human_choice.json`、哈希绑定的 `decision_packet.json`、离线自包含 `construction_review.html`。摘要绑定来源、恢复审阅包、政策、站位图证据和实际物理轨迹；门禁检测文件变化，并禁止手改成功标志放行。

## 当前结果

episode 14 的原始 A 点，经左臂、h=.738、grasp 794 严格抓取成功。因此筛选完成，但候选未选中；场景零修改，新物理试验零次，使用已有轨迹重新验收。没有导航、绕行闭环或最终 Case 1 标签。

保持原样筛选时，下一步需扩大其他原始 episode 的来源审计和筛选。现有来源与稳定证据入口仅支持已审计的 episode 14，批量新来源筛选尚未实现；不能重复恢复 episode 14 来冒充新困难候选。

## 后续 robot_only 分支（当前使用）

“不改场景”的人工选择只绑定原样筛选那一次运行，**不是永久限制**。用户后续明确要求从站位图失败/超范围点建立困难起点，已记录为新的 `robot_only` 方式选择。旧方式和筛除结果保留，当前分支由 `latest.json` 指向；后续目标/家具修改仍可作为新的独立构造分支实施。

```bash
lastmile_pipeline/bin/lastmile construct-robot-start --case-id case1-cup-001 --run-id robot_move_v3
lastmile_pipeline/bin/lastmile check-construct-gate --case-id case1-cup-001
```

当前构造已执行，第一条不必重复，已有目录时拒绝覆盖。仅允许改变机器人初始 base，其他初始 qpos/qvel 必须逐项与源快照一致。筛选从步骤 3 冻结候选坐标获取，不由 agent 猜坐标；首次只接受 `outside_contact_bound`，避免把配置级执行失败或预算耗尽升级成整个位姿不可达。

导航地图使用各机器人身体的真实碰撞 geom 保守 AABB，按高度与环境碰撞 geom 做 Minkowski 过滤，包含头和双臂。不能用单个 .34m 圆盘代表整个机器人。A* 禁止对角穿角，路线简化仍检查自由空间；转向在额外安全停靠点完成，再接近 A。该停靠点是导航中间点，不是另造失败起点。

P131 距目标 1.556593 m，接触上界 1.217127 m，余量 0.339466 m；原始 head_camera 中目标实例可见 111 像素。构造通过只是步骤 5 的候选门禁，不能提前贴闭环成功标签。
