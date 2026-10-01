# 源码职责索引

源码分为物理后端与 workflow 编排。保留已有模块名和导入路径，本次不进行业务代码重构。

## 物理后端：`lastmile/`

| 职责 | 模块 |
|---|---|
| MuJoCo 场景与状态操作 | `simulation.py` |
| cuRobo 抓取校准与执行 | `calibration_runner.py` |
| A*、碰撞网格与底盘控制 | `navigation.py` |
| 动作布局与坐标变换 | `protocol.py` |
| 严格抓取及动作前缀判定 | `validation.py` |
| 接触区域与把手语义 | `parts.py` |
| 资产证据准入 | `admission.py` |
| 来源提取及摘要工具 | `audit.py` |

## 编排、审计与展示：`lastmile/workflow/`

| 职责 | 模块 |
|---|---|
| 公共入口、状态、配置和人工门禁 | `cli.py`、`core.py` |
| 场景恢复、agent 建议、审阅服务 | `restore.py`、`agent.py`、`server.py` |
| A 点稳定抓法 | `stable.py` |
| 站位采样、几何筛选与规划/执行 | `station_map.py`、`station_geometry.py` |
| 站位图审计与展示 | `station_audit.py`、`station_plot.py` |
| 原样筛选与机器人困难起点 | `construction.py`、`hard_start.py` |
| 连续导航抓取 | `closed_loop.py` |
| 闭环审计、图示和视频 | `closed_loop_audit.py`、`closed_loop_plot.py`、`closed_loop_video.py` |
| 最终人工接受与正式包 | `final_review.py`、`case_export.py` |

新增代码优先放到对应职责模块；不要把一次性的探测脚本或报告生成日志放入源码目录。原始执行实现快照保存在对应 case 的审阅/导出目录，不在这里复制维护第二份。
