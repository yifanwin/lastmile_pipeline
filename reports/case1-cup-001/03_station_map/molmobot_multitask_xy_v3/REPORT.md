# Case1 MolmoBot 视觉移动抓取：molmobot_multitask_xy_v3

**运行状态：infrastructure_error。58 个 XY 起点中，成功 0、物理失败 6、无效初始化 23、基础设施异常 1、未运行 28。**

每个有效起点单次试验；这不是每点成功概率，也不是固定底盘可抓取性。旧 case1 交付和人工门禁未修改。

[TOC]

## 执行设置与事先预期

- 本地 MolmoBot-RBY1Multitask，Pick 模式；右腕／头部／左腕 RGB 和22维本体状态，语言指令 `pick up the cup`。目标世界坐标、oracle 抓法和导航路径不输入策略。
- 完整20维动作，头部固定；10 Hz 策略，20 ms 控制周期，4 ms 物理采样；16步预测、8步执行，最多40秒。seed=0，动作限幅有原始输出与执行目标记录。
- 每次恢复原快照，仅改变底盘；A 保留原朝向，其余正对目标。碰撞初始化跳过；原抓取距离界外点不筛除。
- 预期覆盖58点、约35次有效试验；抓取结果事先未知。做成标准是完成有效视觉输入到真实控制和可重放物理验收的证据链，不是预设必须成功。
- 抓取成功仍要求同一夹爪双指真实受力、抬升≥5 cm、无支撑连续保持≥2 s、无禁止碰撞，头部固定、实际关节硬限位合规。
- 本次按用户授权采用躯干反馈告警协议：每步派发目标严格验证 `[0,h,-2h,h,0,0]` 且 h∈[0,0.738]；实际联动残差／范围偏移只警告，不结束 episode。此项与旧严格反馈协议不同，不能作等协议比较。
- 躯干反馈策略：`warn`；有警告试验 6，警告类别采样累计 11617，最大残差 0.021030 rad（原始4 ms轨迹可重算）。

## 起点结果与实际路线

![起点结果与实测底盘轨迹](figures/start_results.png)

图：观测单位为一个 XY 起点的单次试验。圆点为起点，三角为实测终点，叉为无效初始化；灰色障碍投影仅用于显示。没有误差棒，不把未运行或无效点填成失败。

![逐点实测路线](figures/measured_routes.png)

图：各子图给出独立恢复后连续执行的实测底盘路线；不是规划路线。

## 逐点证据与旧 oracle（仅描述性并列）

| 起点 | VLA状态 | 原因 | 旧固定底盘相同位姿 | 视频 |
|---|---|---|---|---|
| A | executed_failure | illegal_collision | verified_success | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v3/trials/A_attempt_000/rollout.mp4) |
| P002 | executed_failure | horizon_exhausted_lift_contact_hold_or_slip | 未实测该位姿 | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v3/trials/P002_attempt_000/rollout.mp4) |
| P005 | executed_failure | illegal_collision | verified_success | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v3/trials/P005_attempt_000/rollout.mp4) |
| P008 | executed_failure | illegal_collision | executed_failure | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v3/trials/P008_attempt_000/rollout.mp4) |
| P011 | executed_failure | illegal_collision | 未实测该位姿 | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v3/trials/P011_attempt_000/rollout.mp4) |
| P014 | invalid_initialization | initial_pose_collision | 未实测该位姿 | — |
| P017 | invalid_initialization | initial_pose_collision | 未实测该位姿 | — |
| P020 | executed_failure | illegal_collision | 未实测该位姿 | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v3/trials/P020_attempt_000/rollout.mp4) |
| P023 | infrastructure_error | KeyboardInterrupt | 未实测该位姿 | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v3/trials/P023_attempt_000/rollout.mp4) |
| P026 | not_run | not_run | 未实测该位姿 | — |
| P029 | invalid_initialization | initial_pose_collision | 未实测该位姿 | — |
| P032 | invalid_initialization | initial_pose_collision | 未实测该位姿 | — |
| P035 | invalid_initialization | initial_pose_collision | 未实测该位姿 | — |
| P038 | not_run | not_run | 未实测该位姿 | — |
| P041 | not_run | not_run | 未实测该位姿 | — |
| P044 | not_run | not_run | 未实测该位姿 | — |
| P047 | invalid_initialization | initial_pose_collision | 未实测该位姿 | — |
| P050 | invalid_initialization | initial_pose_collision | 未实测该位姿 | — |
| P053 | invalid_initialization | initial_pose_collision | 未实测该位姿 | — |
| P056 | not_run | not_run | 未实测该位姿 | — |
| P059 | not_run | not_run | 未实测该位姿 | — |
| P062 | not_run | not_run | 未实测该位姿 | — |
| P065 | not_run | not_run | 未实测该位姿 | — |
| P068 | invalid_initialization | initial_pose_collision | 未实测该位姿 | — |
| P071 | invalid_initialization | initial_pose_collision | 未实测该位姿 | — |
| P074 | invalid_initialization | initial_pose_collision | 未实测该位姿 | — |
| P077 | invalid_initialization | initial_pose_collision | 未实测该位姿 | — |
| P080 | invalid_initialization | initial_pose_collision | 未实测该位姿 | — |
| P083 | invalid_initialization | initial_pose_collision | 未实测该位姿 | — |
| P086 | invalid_initialization | initial_pose_collision | 未实测该位姿 | — |
| P089 | invalid_initialization | initial_pose_collision | 未实测该位姿 | — |
| P092 | invalid_initialization | initial_pose_collision | 未实测该位姿 | — |
| P095 | invalid_initialization | initial_pose_collision | 未实测该位姿 | — |
| P098 | invalid_initialization | initial_pose_collision | 未实测该位姿 | — |
| P101 | invalid_initialization | initial_pose_collision | 未实测该位姿 | — |
| P104 | invalid_initialization | initial_pose_collision | 未实测该位姿 | — |
| P107 | invalid_initialization | initial_pose_collision | 未实测该位姿 | — |
| P110 | invalid_initialization | initial_pose_collision | 未实测该位姿 | — |
| P113 | not_run | not_run | 未实测该位姿 | — |
| P116 | not_run | not_run | 未实测该位姿 | — |
| P119 | not_run | not_run | 未实测该位姿 | — |
| P122 | not_run | not_run | 未实测该位姿 | — |
| P125 | not_run | not_run | verified_success | — |
| P128 | not_run | not_run | 未实测该位姿 | — |
| P131 | not_run | not_run | 未实测该位姿 | — |
| P134 | not_run | not_run | 未实测该位姿 | — |
| P137 | not_run | not_run | 未实测该位姿 | — |
| P140 | not_run | not_run | 未实测该位姿 | — |
| P143 | not_run | not_run | 未实测该位姿 | — |
| P146 | not_run | not_run | 未实测该位姿 | — |
| P149 | not_run | not_run | 未实测该位姿 | — |
| P152 | not_run | not_run | 未实测该位姿 | — |
| P155 | not_run | not_run | 未实测该位姿 | — |
| P158 | not_run | not_run | budget_exhausted_no_solution | — |
| P161 | not_run | not_run | 未实测该位姿 | — |
| P164 | not_run | not_run | 未实测该位姿 | — |
| P167 | not_run | not_run | 未实测该位姿 | — |
| P170 | not_run | not_run | 未实测该位姿 | — |

## 结果边界与下一步

- 物理失败原因计数：`{'illegal_collision': 5, 'horizon_exhausted_lift_contact_hold_or_slip': 1}`。这是事实分类，不是已验证的因果归因。
- baseline 是固定底盘、冻结 grasp 的 oracle，与本次允许移动且双臂可动的 VLA 不等条件；未做整体胜率或显著性比较。
- 每点只有一次且全部使用同一模型与seed，不做概率、独立样本或泛化声明。原始相机／模型输入、模型动作、限幅、物理接触和轨迹均可追溯。
- 无效初始化不支持判断模型能力；基础设施异常／未运行点不支持判断物理失败。若批次中断，修复公共接口后必须按冻结配置续跑；代码变化需新run。
- 尚未用对照干预验证失败机制；不因失败放宽碰撞、抬升或保持阈值。下一步是否重复试验或更换协议，应另行决定。

## 证据与验证

- [冻结来源与配置](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v3/frozen_inputs.json) · [VLA协议](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v3/vla_protocol.json) · [逐点结果](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v3/results.json) · [运行状态](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v3/run_state.json)。
- 每个完成试验的 `audit.json` 从4 ms压缩原始轨迹重算结论；`evidence_manifest.json` 保存输入、动作、轨迹、视频与快照摘要。续跑检查摘要和重新验收。
- 保留：证据图PNG/SVG及复现脚本、紧凑CSV、冻结配置、动作和轨迹。权重只读引用；模型输入、快照、视频与完整日志不加入Git。
- 视觉规划：保留起点结果图和逐点实测路线图（证据类），不采用平滑概率热图或装饰性统计图。HTML使用同一Markdown生成，图片内嵌，可离线阅读。
