# Lastmile 半自动构造管线

以《lastmile case 构建方案.md》第六节为中心。当前实现 **步骤 1–3、步骤 4 的原样筛选/仅移动机器人分支，以及步骤 5 的 A* 连续导航抓取验收**。步骤 6 最终人工接受与导出已实现，当前 case 已获用户明确接受。原始 A 点成功，不能作为困难起点；新困难起点从步骤 3 超范围候选选择。自动生成证据，人工决定场景是否保留，人工不能覆盖物理失败。

## 当前交付

新增 [独立 MolmoBot VLA 起点测试](docs/stages/vla_station_test.md)：`bin/lastmile vla-station-test`。
完整20维视觉移动抓取，58个XY独立记录；不覆盖下列旧交付或六阶段门禁。
最新 [MolmoBot v4 报告](reports/case1-cup-001/03_station_map/molmobot_multitask_xy_v4/REPORT.md) · [离线 HTML](reports/case1-cup-001/03_station_map/molmobot_multitask_xy_v4/REPORT.html)：按用户授权，躯干反馈与轻微碰撞仅告警；35次实跑，5次抓起成功、27次超时、3次严重穿模停止，23个碰撞初始化跳过。
历史 [MolmoBot v2 报告](reports/case1-cup-001/03_station_map/molmobot_multitask_xy_v2/REPORT.md) · [离线 HTML](reports/case1-cup-001/03_station_map/molmobot_multitask_xy_v2/REPORT.html)：35次实跑全部因躯干联动约束提前终止，23个碰撞起点跳过；不能仅据此判断模型抓杯能力。

- [Case 1 交付文档（含站位图）](reports/delivery/case1-cup-001.md)
- [P131 → A 闭环视频](cases/case1-cup-001/05_closed_loop/runs/astar_v2/delivery/case1_r_closed_loop.mp4)

3 / 3 次连续试验成功，已获人工接受。最终交付仅提供文档及必要图片、视频，不要求打包或阅读 SHA 检查材料。既有工程归档保留。

## 已生成的候选

- [场景交互审阅卡](cases/case1-cup-001/01_restore_review/review.html)：episode 14 / house 103 / Cup_30 / DiningTable。
- [流程状态](cases/case1-cup-001/manifest.json)。人工已批准 north/south/west，步骤 3 粗站位图已完成。
- [粗站位图](cases/case1-cup-001/03_station_map/runs/coarse_v1/station_map.html) · [PNG](cases/case1-cup-001/03_station_map/runs/coarse_v1/station_map.png)。
- [本次执行报告](reports/REPORT.md)。A 指原 episode 的成功候选站位，不是困难起点。

## 目录

```text
bin/lastmile                 唯一推荐命令入口
configs/                    分阶段配置、动作协议、只读 agent 配置
src/lastmile/workflow/       编排、人工门禁、审阅服务、agent 建议
src/lastmile/               MuJoCo / cuRobo / 接触验收后端
cases/<case-id>/
  manifest.json             六阶段状态（自动结果与人工决定分开）
  inputs/                   有来源校验的 episode、资产与执行上下文
  01_restore_review/        恢复快照、四视图、审阅包、人工记录、agent 建议
  02_stable_grasp/          稳定 grasp、历史轨迹复核、新 A 点实跑
  03_station_map/           候选、双臂五高度规划、独立物理试验和站位图
  04_construct_start/       原样筛选历史、按运行记录的构造方式和派生起点
  05_closed_loop/           A* 连续导航、实测到站抓取、重复验收和视频
  06_export_case/           最终审阅、人工决定、草稿、正式包与回执
docs/stages/                各阶段输入、输出和通过条件
reports/                    当前交付报告和执行日志
archive/calibration_v1/     旧校准配置、报告和原始轨迹，只读历史证据
```

报告已按用途和阶段拆分：`reports/delivery/` 放最终交付说明，`reports/overview/` 放实现总结，`reports/<case-id>/<阶段>/` 放阶段报告、`checks/` 检查记录和 `logs/` 日志。测试按 `physics/`、`navigation/`、`workflow/` 分组。

- [文档索引](docs/README.md) · [目录结构说明](docs/architecture/directory-layout.md)
- [报告索引](reports/README.md) · [仓库实现总结](reports/overview/repository-summary.md)
- [源码职责索引](src/README.md) · [配置说明](configs/README.md) · [测试说明](tests/README.md)

原工程、benchmark、源 XML 和资产不改；旧材料保留在 archive，不继续维护旧 scripts 为公共入口。历史 JSON 的旧路径由 workflow 映射，不修改原始轨迹。

## 操作

从工程根目录执行；默认 CUDA 5，也可 `CUDA_VISIBLE_DEVICES=0`。入口复用 `molmospaces/.venv` 和环境脚本。

```bash
# 新候选：已审计来源中的 episode 14；同一审阅包不能覆盖
lastmile_pipeline/bin/lastmile restore --case-id case1-cup-002 --source-index 14
# 可选：向 .env 指定 API 发送三张图和场景事实，需要外发授权
lastmile_pipeline/bin/lastmile agent-review --case-id case1-cup-002
# 尚未人工确认时可预校准，但不会自动放行步骤 3
lastmile_pipeline/bin/lastmile calibrate-a --case-id case1-cup-002 --provisional
# 人工审阅（仅监听本机）
lastmile_pipeline/bin/lastmile serve-review --case-id case1-cup-001 --port 8765
# 浏览器打开 http://127.0.0.1:8765/review.html，填写后批准或拒绝
```

远程机器可 `ssh -L 8765:127.0.0.1:8765 <server>`。也可离线打开 HTML，填写后下载 JSON，再导入：

```bash
lastmile_pipeline/bin/lastmile approve --case-id case1-cup-001 --file /path/to/case1-cup-001-human-review.json
lastmile_pipeline/bin/lastmile status --case-id case1-cup-001
lastmile_pipeline/bin/lastmile check-gate --case-id case1-cup-001
```

审阅必须确认目标、桌面、适合桌边、保留纯距离型；无绕行可能就拒绝并换场景，不删墙补救。`check-gate` 只检查下一阶段条件，**不执行站位图任务本身**。

## 步骤 3：粗站位图

```bash
lastmile_pipeline/bin/lastmile station-map --case-id case1-cup-001 --run-id coarse_v1
# 相同冻结配置中断续跑
lastmile_pipeline/bin/lastmile station-map --case-id case1-cup-001 --run-id coarse_v1 --resume
```

[阶段说明](docs/stages/03_station_map.md)。按人工批准的桌边采样含朝向的底盘位姿，几何筛选后检查双臂 × 五个 h，关键候选恢复同一原始快照做真实抓取。只放置机器人底盘，不移目标或家具；这不是导航。真实成功、规划通过、几何过滤和搜索预算耗尽分开显示。局部 Gaussian 插值只采用完整动态 Pick 标签，未知区域不填 0。

## 物理证据与限制

抓取用资产原 grasp 坐标及缩放，左臂、躯干 h=0.738、grasp row 794。cuRobo 执行预抓取→接近→闭合→抬升→保持。严格要求双指真实受力接触、抬升至少 5 cm、无支撑持续至少 2 s，并检查基座、躯干、头和闲置臂约束。

步骤 2 复核旧 5 次名义 / 20 次 ±1 cm、±2° 基座扰动原始轨迹，再做当前模型新 A 点实跑。旧数据不冒充新试验，不推广到其他站位或导航。当前稳定证据导入只支持 episode 14 / Cup_30；恢复入口只接受已审计来源。新增来源必须先审计，不猜资产名。

`.env` 仅读取 `LLM_API_KEY / LLM_BASE_URL / LLM_MODEL`，不复制凭据；agent 只有建议权，无批准权。服务 API 不调用 agent，不执行 agent 生成的代码或坐标。

```bash
PYTHONPATH=lastmile_pipeline/src molmospaces/.venv/bin/python -m unittest discover -s lastmile_pipeline/tests -v
```

## 步骤 4：仅筛选原始起点

用户已选择不改场景；episode 14 原始起点通过严格 Pick，不能充当失败起点。筛选完成但未选中困难候选，步骤 5 不放行。

[步骤说明](docs/stages/04_construct_start.md) · [审阅卡](cases/case1-cup-001/04_construct_start/construction_review.html) · [执行报告](reports/case1-cup-001/04_construct_start/REPORT.md)

```bash
MUJOCO_GL=disable lastmile_pipeline/bin/lastmile check-construct-gate --case-id case1-cup-001
# 当前预期退出 1：原始起点已成功抓取
```

## 当前构造与连续执行

“不改场景”是原样筛选那一次的临时选择，并非永久禁止修改。后续按用户明确请求切换到 `robot_only`，旧记录保留。当前 P131 起点只改机器人 base，目标/家具未变；未来其他场景修改需单独保留来源、修改清单和验收。

[步骤 4 新分支说明](docs/stages/04_construct_start.md) · [步骤 5 说明](docs/stages/05_closed_loop.md)
