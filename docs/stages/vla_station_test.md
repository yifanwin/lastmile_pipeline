# 独立 VLA 起点移动抓取测试

这是 case1 的额外模型评测，不是六阶段门禁或旧固定底盘站位图的替代。
原工程、case manifest、人工接受记录和旧 run 保持不变。

```bash
CUDA_VISIBLE_DEVICES=5 lastmile_pipeline/bin/lastmile vla-station-test \
  --case-id case1-cup-001 --station-run-id coarse_v1 \
  --run-id molmobot_multitask_xy_v4 \
  --checkpoint-path /nas/wenyifan/models/MolmoBot-RBY1Multitask
# 仅配置、权重、来源和执行代码未变时可续跑；不重跑已经完成的点。
CUDA_VISIBLE_DEVICES=5 lastmile_pipeline/bin/lastmile vla-station-test --run-id molmobot_multitask_xy_v4 --resume
```

复用 MolmoSpaces 仿真环境；自动以 MolmoBot 虚拟环境启动仅监听本机的推理子进程。
需要 CUDA、EGL、本地 tokenizer 缓存；设置离线模式，不下载或更改权重。

## 输入与动作

- 从 coarse_v1 选择 A 与零 yaw 偏移候选，共58个XY。A 保持原朝向；其余朝向目标。
- 初始化恢复原全状态，仅设置底盘三个坐标并清零底盘速度；非底盘状态／速度逐次审计。无效碰撞或地面支撑不足点记录并跳过；不按原 reach 界限筛除。
- 右腕、头部、左腕 RGB；1024×576，名义FOV为58°／139°／58°，姿态沿用冻结恢复模型，无新增随机化。头部使用上游鱼眼预处理。语言为 `pick up the cup`，22维实测状态中躯干取 `[1,2,3]`。模型不接收目标坐标、oracle抓法、点提示或规划路径。
- 完整20维控制遵循根目录 `RBY-1自由度控制.md`。头部保持初始角度。底盘和双臂相对实测关节值设目标，不在物理子步重新累加。
- Pick模式显式关闭服务端二值夹爪阈值；原始输出（可能±100）按本地位置伺服夹爪范围 `[-0.05,0]` 限幅，不能作为力矩下发。躯干标量限制在 `[0,.738]`；执行器目标按模型限位限幅。原始动作、限幅和执行目标全部记录，非有限动作／错误维度停止批次。
- 10 Hz策略、20 ms控制、4 ms仿真；16步预测，执行8步后重新推理。最多400策略步／40秒。每试验seed=0，独立清空模型缓存。策略动作可使底盘和双臂同时运动。

## 真实成功与失败

当前独立 `lastmile-visual-mobile-pick-v3` 协议保持旧抬升、保持、滑移、头部阈值，不应用旧固定底盘／闲置臂约束；躯干反馈按用户授权只警告（详见末节）。
连续执行中禁止恢复、瞬移、修改对象或用规划器救场。严重穿模、头部漂移和实际关节硬限位越界仍为物理失败；派发联动指令错误为接口异常。
任一同一夹爪双指真实受力接触、抬升≥5cm、无支撑连续≥2s且相对滑移合规才成功。
躯干实际联动残差保留审计但不终止，正确派发目标仍严格验证。

结果分 `verified_success`、`executed_failure`、`invalid_initialization`、`infrastructure_error`、`not_run`。
无效初始化和基础设施异常没有二值失败标签。A物理失败不阻止其他点；公共模型、相机或执行接口异常停止批次。
一个XY只做一次；结果不是每点成功概率，也不是固定站位的抓取能力。

## 证据与输出

新证据位于 `cases/<case-id>/03_station_map/runs/<run-id>/`。
冻结来源、审批、物理模型、权重、配置和执行源码摘要；独立保存VLA协议、候选、相机校准与逐点结果。
每次attempt保存初始化快照、4ms压缩JSONL轨迹、模型实际输入、16×20原始预测、动作与限幅记录、10Hz状态、四视图视频及哈希清单。
`audit.json` 使用相同状态机从原轨迹重算结论；续跑先检查已完成证据。未完成attempt保留，新的attempt从原快照运行。

报告在 `reports/<case-id>/03_station_map/<run-id>/`，包括 Markdown、自包含HTML、结果CSV、SVG/PNG起点图和实测路线图。
旧oracle只有完全相同位姿有物理证据时才描述性并列；不作等条件胜率比较。
所有轨迹、模型输入、快照、视频、完整日志不加入Git；只保留小配置、审计记录、最终图和报告。

```bash
PYTHONPATH=lastmile_pipeline/src molmospaces/.venv/bin/python -m unittest discover -s lastmile_pipeline/tests -v
```

单元测试不能替代本次真实模型／仿真验收。读取报告时先看运行状态及未运行数量。

## 本次执行结果

`molmobot_multitask_xy_v1` 在本机 WebSocket 代理握手阶段失败，没有物理执行。
修复本机连接的代理选择后，`molmobot_multitask_xy_v2` 完成58点记录、35次有效实跑和23次无效初始化跳过。
35次均因实际躯干联动超过旧协议0.002rad阈值提前终止；命令始终符合1D联动。没有严格抓取通过，不能把早停结果当作完整40秒模型抓取能力结论。

- [执行报告与诊断](../../reports/case1-cup-001/03_station_map/molmobot_multitask_xy_v2/REPORT.md)
- [离线 HTML](../../reports/case1-cup-001/03_station_map/molmobot_multitask_xy_v2/REPORT.html)
- [35次原轨迹重验回执](../../reports/case1-cup-001/03_station_map/molmobot_multitask_xy_v2/evidence_revalidation.json)

本次117项回归测试通过。新代码与v1冻结源码不同，不能用v1续跑；v2已完成。
图和诊断可先运行报告目录 `figures/start_results.py`，再运行 `figures/torso_diagnostic.py` 复现。


## 躯干反馈告警协议（用户授权，v3）

用户明确要求正确派发躯干动作后，实际联动残差、范围偏移只告警、不终止 episode。
配置 `torso_feedback_policy: warn` 对应独立 VLA v2 协议；每步派发前验证完整六关节目标
`[0,h,-2h,h,0,0]`、h 限制 `[0,0.738]`，如执行器限幅破坏联动则不派发任何控制并停止公共批次。
反馈警告保留首次／末次时间、采样数、最大残差，原始4 ms轨迹供重新验收。
碰撞、实际关节硬限位、头部固定、非有限状态以及抓取双指真实受力／抬升／2秒保持／滑移条件不变。
旧 v1/v2 run 和六阶段协议不修改；旧轨迹继续使用默认 `strict` 验收，不能用告警协议改写旧结论。
新实验 run-id `molmobot_multitask_xy_v3`；预期58记录、23无效初始化、35有效试验，结果事先未知。


## 轻微碰撞告警协议（用户授权，v4）

用户进一步要求轻微碰撞不终止，未严重穿模时继续实验。
新 VLA v3 协议配置 `collision_feedback_policy: warn_minor`、`severe_penetration_m: 0.02`。
非指尖目标接触／自碰撞／环境接触沿用原检测门槛，但穿透深度小于2cm时仅记录警告；
任何机器人接触（含允许的指尖接触和底盘地面接触）达到或超过2cm则终止为 `severe_penetration`。保存每个接触对、深度和法向受力，逐点统计警告。
2cm为本次明确声明的工程默认阈值，不等于真实硬件安全保证，也不代表旧严格无碰撞验收通过。
初始化仍按原计划跳过碰撞位置，避免修改58点／35有效点的覆盖定义。
目标双指真实受力、无支撑、抬升和保持／滑移验收以及实际关节硬限位和头部固定不变。
原 v3 批次因用户协议更新主动中断，保留完成结果和中断证据；新 `molmobot_multitask_xy_v4`
从原始快照重新执行全部有效点，不续跑或重判旧结果。预期结果未知，完整覆盖或公共接口故障停止。


### v4 全覆盖实际结果

`molmobot_multitask_xy_v4` 已完成58个XY记录：23个无效碰撞初始化、35次真实执行。
5次成功（P128、P146、P149、P167、P170，均为右手），27次完整40秒未满足抓取条件，3次严重穿模停止。
35次均出现躯干反馈警告，24次有轻微碰撞警告；派发联动验证严格保留。
124项回归测试通过。报告与逐点录像见对应 run 独立报告目录；旧门禁和旧证据不变。
复现报告先运行 `figures/start_results.py`，再运行 `figures/evaluation_diagnostic.py`。
