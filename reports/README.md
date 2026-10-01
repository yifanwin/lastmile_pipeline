# 报告索引

## 优先阅读

- [当前交付状态](REPORT.md)
- [仓库实现总结](overview/repository-summary.md)
- [Case 1 用户交付说明](delivery/case1-cup-001.md)
- [目录整理总结](maintenance/2026-10-02-directory-organization.md)

## case1-cup-001 阶段记录

| 阶段 | 报告 / 记录 | 检查记录 | 日志 |
|---|---|---|---|
| 1. 恢复与审阅 | [步骤 1–2 联合历史报告](case1-cup-001/01_restore_review/REPORT.md) | [checks](case1-cup-001/01_restore_review/checks/) | [logs](case1-cup-001/01_restore_review/logs/) |
| 2. 稳定抓法 | 见步骤 1–2 联合报告 | 物理证据保存在 case 目录 | [logs](case1-cup-001/02_stable_grasp/logs/) |
| 3. 站位图 | [报告](case1-cup-001/03_station_map/REPORT.md) · [可视化规划](case1-cup-001/03_station_map/visual-plan.md) | [checks](case1-cup-001/03_station_map/checks/) | [logs](case1-cup-001/03_station_map/logs/) |
| 4. 困难起点 | [原样筛选历史报告](case1-cup-001/04_construct_start/REPORT.md) | [checks](case1-cup-001/04_construct_start/checks/) | [logs](case1-cup-001/04_construct_start/logs/) |
| 5. 连续导航抓取 | [报告](case1-cup-001/05_closed_loop/REPORT.md) · [可视化规划](case1-cup-001/05_closed_loop/visual-plan.md) | [checks](case1-cup-001/05_closed_loop/checks/) | [logs](case1-cup-001/05_closed_loop/logs/) |
| 6. 接受与导出 | [报告](case1-cup-001/06_export_case/REPORT.md) | [checks](case1-cup-001/06_export_case/checks/) | [logs](case1-cup-001/06_export_case/logs/) |

历史报告保留生成当时的结论，不代表最新阶段状态。最新状态查看 [case manifest](../cases/case1-cup-001/manifest.json)。

新增案例使用新的 `reports/<case-id>/`；日志与检查结果分别进入阶段的 `logs/`、`checks/`。不要把原始物理证据复制到这里。
