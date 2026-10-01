# Lastmile 半自动构造管线

以《lastmile case 构建方案.md》第六节为中心。当前实现 **Case 1 的步骤 1–2**；步骤 3–6 只有目录与门禁契约，尚未实现。自动生成证据，人工决定场景是否保留，人工不能覆盖物理失败。

## 已生成的候选

- [场景交互审阅卡](cases/case1-cup-001/01_restore_review/review.html)：episode 14 / house 103 / Cup_30 / DiningTable。
- [流程状态](cases/case1-cup-001/manifest.json)。人工仍待确认。
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
  03_station_map/           后续站位图（预留）
  04_construct_start/       后续人工选构造方式（预留）
  05_closed_loop/           后续起点失败和绕行抓取证据（预留）
  06_export_case/           后续最终人工确认和导出（预留）
docs/stages/                各阶段输入、输出和通过条件
reports/                    当前交付报告和执行日志
archive/calibration_v1/     旧校准配置、报告和原始轨迹，只读历史证据
```

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

审阅必须确认目标、桌面、适合桌边、保留纯距离型；无绕行可能就拒绝并换场景，不删墙补救。`check-gate` 只检查下一阶段条件，**不执行尚未实现的步骤 3**。

## 物理证据与限制

抓取用资产原 grasp 坐标及缩放，左臂、躯干 h=0.738、grasp row 794。cuRobo 执行预抓取→接近→闭合→抬升→保持。严格要求双指真实受力接触、抬升至少 5 cm、无支撑持续至少 2 s，并检查基座、躯干、头和闲置臂约束。

步骤 2 复核旧 5 次名义 / 20 次 ±1 cm、±2° 基座扰动原始轨迹，再做当前模型新 A 点实跑。旧数据不冒充新试验，不推广到其他站位或导航。当前稳定证据导入只支持 episode 14 / Cup_30；恢复入口只接受已审计来源。新增来源必须先审计，不猜资产名。

`.env` 仅读取 `LLM_API_KEY / LLM_BASE_URL / LLM_MODEL`，不复制凭据；agent 只有建议权，无批准权。服务 API 不调用 agent，不执行 agent 生成的代码或坐标。

```bash
PYTHONPATH=lastmile_pipeline/src molmospaces/.venv/bin/python -m unittest discover -s lastmile_pipeline/tests -v
```
