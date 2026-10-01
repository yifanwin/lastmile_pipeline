# 配置目录

| 配置 | 用途 |
|---|---|
| `01_restore_review.yaml` | 来源、恢复缓存、相机与审阅策略 |
| `02_stable_grasp.yaml` | A 点稳定抓法和历史证据导入 |
| `03_station_map.yaml` | 候选采样、双臂五高度规划、物理试验和插值 |
| `04_construct_start.yaml` | 原始起点筛选政策 |
| `04_robot_start.yaml` | 只移动机器人起点的构造及闭环参数 |
| `06_export_case.yaml` | 最终人工接受、重复试验要求和导出政策 |
| `agents/scene_reviewer.yaml` | 仅提供场景建议的 agent 配置 |
| `protocols/protocol_v1.json` | 动作布局、合法范围、物理阈值和准入规则 |
| `cases/` | 预留案例专属配置，当前无配置文件 |

当前没有独立步骤 5 YAML，其相关参数由 `04_robot_start.yaml` 提供。阶段文件已有数字前缀，且原路径绑定现有证据，本次不移动或复制。

凭据不放在这里。改动冻结配置后必须使用新的运行/审阅标识；不要为了调整目录或让测试通过而改验收阈值。
