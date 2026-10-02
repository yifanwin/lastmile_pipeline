# Case1 MolmoBot 视觉移动抓取：molmobot_multitask_xy_v2

**运行状态：complete。58 个 XY 起点中，成功 0、物理失败 35、无效初始化 23、基础设施异常 0、未运行 0。**

每个有效起点单次试验；这不是每点成功概率，也不是固定底盘可抓取性。旧 case1 交付和人工门禁未修改。

**注意：35次均为躯干联动约束提前终止（0.068–7.344秒），不是完整40秒抓取尝试。不能仅凭本次结果判定模型没有抓杯能力。**

[TOC]

## 执行设置与事先预期

- 本地 MolmoBot-RBY1Multitask，Pick 模式；右腕／头部／左腕 RGB 和22维本体状态，语言指令 `pick up the cup`。目标世界坐标、oracle 抓法和导航路径不输入策略。
- 完整20维动作，头部固定；10 Hz 策略，20 ms 控制周期，4 ms 物理采样；16步预测、8步执行，最多40秒。seed=0，动作限幅有原始输出与执行目标记录。
- 每次恢复原快照，仅改变底盘；A 保留原朝向，其余正对目标。碰撞初始化跳过；原抓取距离界外点不筛除。
- 预期覆盖58点、约35次有效试验；抓取结果事先未知。做成标准是完成有效视觉输入到真实控制和可重放物理验收的证据链，不是预设必须成功。
- 严格成功要求同一夹爪双指真实受力、抬升≥5 cm、无支撑连续保持≥2 s、无禁止碰撞，且头部、躯干联动与限位合规。

## 起点结果与实际路线

![起点结果与实测底盘轨迹](figures/start_results.png)

图：观测单位为一个 XY 起点的单次试验。圆点为起点，三角为实测终点，叉为无效初始化；灰色障碍投影仅用于显示。没有误差棒，不把未运行或无效点填成失败。

![逐点实测路线](figures/measured_routes.png)

图：各子图给出独立恢复后连续执行的实测底盘路线；不是规划路线。

## 逐点证据与旧 oracle（仅描述性并列）

| 起点 | VLA状态 | 原因 | 旧固定底盘相同位姿 | 视频 |
|---|---|---|---|---|
| A | executed_failure | torso_linkage | verified_success | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2/trials/A_attempt_000/rollout.mp4) |
| P002 | executed_failure | torso_linkage | 未实测该位姿 | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2/trials/P002_attempt_000/rollout.mp4) |
| P005 | executed_failure | torso_linkage | verified_success | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2/trials/P005_attempt_000/rollout.mp4) |
| P008 | executed_failure | torso_linkage | executed_failure | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2/trials/P008_attempt_000/rollout.mp4) |
| P011 | executed_failure | torso_linkage | 未实测该位姿 | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2/trials/P011_attempt_000/rollout.mp4) |
| P014 | invalid_initialization | initial_pose_collision | 未实测该位姿 | — |
| P017 | invalid_initialization | initial_pose_collision | 未实测该位姿 | — |
| P020 | executed_failure | torso_linkage | 未实测该位姿 | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2/trials/P020_attempt_000/rollout.mp4) |
| P023 | executed_failure | torso_linkage | 未实测该位姿 | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2/trials/P023_attempt_000/rollout.mp4) |
| P026 | executed_failure | torso_linkage | 未实测该位姿 | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2/trials/P026_attempt_000/rollout.mp4) |
| P029 | invalid_initialization | initial_pose_collision | 未实测该位姿 | — |
| P032 | invalid_initialization | initial_pose_collision | 未实测该位姿 | — |
| P035 | invalid_initialization | initial_pose_collision | 未实测该位姿 | — |
| P038 | executed_failure | torso_linkage | 未实测该位姿 | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2/trials/P038_attempt_000/rollout.mp4) |
| P041 | executed_failure | torso_linkage | 未实测该位姿 | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2/trials/P041_attempt_000/rollout.mp4) |
| P044 | executed_failure | torso_linkage | 未实测该位姿 | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2/trials/P044_attempt_000/rollout.mp4) |
| P047 | invalid_initialization | initial_pose_collision | 未实测该位姿 | — |
| P050 | invalid_initialization | initial_pose_collision | 未实测该位姿 | — |
| P053 | invalid_initialization | initial_pose_collision | 未实测该位姿 | — |
| P056 | executed_failure | torso_linkage | 未实测该位姿 | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2/trials/P056_attempt_000/rollout.mp4) |
| P059 | executed_failure | torso_linkage | 未实测该位姿 | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2/trials/P059_attempt_000/rollout.mp4) |
| P062 | executed_failure | torso_linkage | 未实测该位姿 | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2/trials/P062_attempt_000/rollout.mp4) |
| P065 | executed_failure | torso_linkage | 未实测该位姿 | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2/trials/P065_attempt_000/rollout.mp4) |
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
| P113 | executed_failure | torso_linkage | 未实测该位姿 | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2/trials/P113_attempt_000/rollout.mp4) |
| P116 | executed_failure | torso_linkage | 未实测该位姿 | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2/trials/P116_attempt_000/rollout.mp4) |
| P119 | executed_failure | torso_linkage | 未实测该位姿 | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2/trials/P119_attempt_000/rollout.mp4) |
| P122 | executed_failure | torso_linkage | 未实测该位姿 | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2/trials/P122_attempt_000/rollout.mp4) |
| P125 | executed_failure | torso_linkage | verified_success | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2/trials/P125_attempt_000/rollout.mp4) |
| P128 | executed_failure | torso_linkage | 未实测该位姿 | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2/trials/P128_attempt_000/rollout.mp4) |
| P131 | executed_failure | torso_linkage | 未实测该位姿 | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2/trials/P131_attempt_000/rollout.mp4) |
| P134 | executed_failure | torso_linkage | 未实测该位姿 | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2/trials/P134_attempt_000/rollout.mp4) |
| P137 | executed_failure | torso_linkage | 未实测该位姿 | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2/trials/P137_attempt_000/rollout.mp4) |
| P140 | executed_failure | torso_linkage | 未实测该位姿 | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2/trials/P140_attempt_000/rollout.mp4) |
| P143 | executed_failure | torso_linkage | 未实测该位姿 | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2/trials/P143_attempt_000/rollout.mp4) |
| P146 | executed_failure | torso_linkage | 未实测该位姿 | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2/trials/P146_attempt_000/rollout.mp4) |
| P149 | executed_failure | torso_linkage | 未实测该位姿 | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2/trials/P149_attempt_000/rollout.mp4) |
| P152 | executed_failure | torso_linkage | 未实测该位姿 | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2/trials/P152_attempt_000/rollout.mp4) |
| P155 | executed_failure | torso_linkage | 未实测该位姿 | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2/trials/P155_attempt_000/rollout.mp4) |
| P158 | executed_failure | torso_linkage | budget_exhausted_no_solution | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2/trials/P158_attempt_000/rollout.mp4) |
| P161 | executed_failure | torso_linkage | 未实测该位姿 | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2/trials/P161_attempt_000/rollout.mp4) |
| P164 | executed_failure | torso_linkage | 未实测该位姿 | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2/trials/P164_attempt_000/rollout.mp4) |
| P167 | executed_failure | torso_linkage | 未实测该位姿 | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2/trials/P167_attempt_000/rollout.mp4) |
| P170 | executed_failure | torso_linkage | 未实测该位姿 | [视频](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2/trials/P170_attempt_000/rollout.mp4) |

## 结果边界与下一步

- 物理失败原因计数：`{'torso_linkage': 35}`。这是事实分类，不是已验证的因果归因。
- baseline 是固定底盘、冻结 grasp 的 oracle，与本次允许移动且双臂可动的 VLA 不等条件；未做整体胜率或显著性比较。
- 每点只有一次且全部使用同一模型与seed，不做概率、独立样本或泛化声明。原始相机／模型输入、模型动作、限幅、物理接触和轨迹均可追溯。
- 无效初始化不支持判断模型能力；基础设施异常／未运行点不支持判断物理失败。若批次中断，修复公共接口后必须按冻结配置续跑；代码变化需新run。
- 尚未用对照干预验证失败机制；不因失败放宽碰撞、抬升或保持阈值。下一步是否重复试验或更换协议，应另行决定。

## 证据与验证

- [冻结来源与配置](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2/frozen_inputs.json) · [VLA协议](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2/vla_protocol.json) · [逐点结果](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2/results.json) · [运行状态](../../../../cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2/run_state.json)。
- 每个完成试验的 `audit.json` 从4 ms压缩原始轨迹重算结论；`evidence_manifest.json` 保存输入、动作、轨迹、视频与快照摘要。续跑检查摘要和重新验收。
- 保留：证据图PNG/SVG及复现脚本、紧凑CSV、冻结配置、动作和轨迹。权重只读引用；模型输入、快照、视频与完整日志不加入Git。
- 视觉规划：保留起点结果图和逐点实测路线图（证据类），不采用平滑概率热图或装饰性统计图。HTML使用同一Markdown生成，图片内嵌，可离线阅读。


<!-- torso-diagnostic -->
## 躯干提前终止诊断

**35/35 次有效试验因实际躯干联动残差超限提前终止；不是完整40秒抓取尝试后的失败。**

- 命令六关节的联动残差最大值为 **0 rad**，符合 `[0,h,-2h,h,0,0]`。模型并未获得额外躯干自由度。
- 实测残差在末帧超过冻结的 **0.002 rad**；终止时间范围 **0.068–7.344 s**，中位数 **0.980 s**。
- 最大目标抬升为 **0.000000 m**；出现过同一夹爪双指同时真实受力的试验数为 **0**。
- 事实：控制目标符合1D联动，实际全身动态响应违反严格联动阈值。推断：当前全身伺服桥／动力学与严格验收之间的不匹配限制了评测。尚未通过干预确认具体是伺服增益、机械耦合、全身反作用或其他因素。
- 因此，本次结论是“当前桥接与冻结协议下没有严格通过”，不能单独据此声称MolmoBot没有视觉抓杯能力。未修改阈值、夹爪或机器人参数补救。

![命令与实际躯干联动诊断](figures/torso_diagnostic.png)

图：左为每个起点单次试验的命令最大残差与实际终止残差；右为A和P008的4ms原始记录。虚线是固定阈值，无误差棒；不是重复随机试验统计。

![A点原始RGB与实际模型头部输入](figures/visual_inputs.png)

图：A点头部、右腕和左腕原始RGB，以及上游鱼眼处理后实际进入模型的头部图像；仅在交付拼图上加标题，不修改模型输入。

[逐点诊断指标](torso_metrics.csv) · [复现脚本](figures/torso_diagnostic.py)

### 后续建议（未执行）

先用独立对照测试全身动态动作时的实际躯干保持／联动，再判断控制桥是否需要修复；不要直接放宽本次阈值，或把早停试验解释为40秒模型能力测试。
