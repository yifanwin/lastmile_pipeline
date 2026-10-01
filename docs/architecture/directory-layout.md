# 目录结构与存放规则

**代码、配置、功能说明、执行记录、最终交付和原始证据分别存放。报告按案例和阶段归类，不再全部放在根目录。**

```text
lastmile_pipeline/
├── README.md / TODO.md / AGENTS.md
├── bin/lastmile                       命令入口
├── src/                              源码；职责索引见 src/README.md
│   └── lastmile/
│       └── workflow/                 六阶段编排、审计与展示
├── configs/                          冻结配置；职责索引见 configs/README.md
│   ├── 01_*.yaml … 06_*.yaml         阶段配置，保留已绑定的原路径
│   ├── agents/                       场景建议 agent 配置
│   ├── protocols/                    动作和验收协议
│   └── cases/                        预留 case 专属配置，当前无配置文件
├── tests/
│   ├── physics/                      动作协议与严格抓取判定
│   ├── navigation/                   路径、导航约束与连续执行参数
│   └── workflow/                     人工门禁、站位图、构造和导出
├── docs/
│   ├── README.md                     文档索引
│   ├── architecture/                 目录规则与迁移清单
│   └── stages/                       六阶段输入、输出、通过条件
├── reports/
│   ├── README.md / REPORT.md          报告导航与当前交付状态
│   ├── overview/                     仓库级实现总结
│   ├── delivery/                     用户最终交付说明
│   ├── maintenance/                  工程整理与维护总结
│   ├── logs/                         仓库级测试日志
│   └── case1-cup-001/
│       ├── 01_restore_review/        含原步骤 1–2 联合报告
│       ├── 02_stable_grasp/
│       ├── 03_station_map/
│       ├── 04_construct_start/
│       ├── 05_closed_loop/
│       └── 06_export_case/
│           ├── REPORT.md             阶段执行报告（有报告的阶段）
│           ├── checks/               JSON 验证记录（按需）
│           └── logs/                 执行日志（按需）
├── cases/<case-id>/                   原始来源、六阶段证据与人工决定
└── archive/calibration_v1/            历史校准、资产登记和原始轨迹
```

## 新文件应该放在哪里

| 文件类型 | 存放位置 |
|---|---|
| 通用功能或阶段规范 | `docs/stages/` |
| 架构、目录或模块职责说明 | `docs/architecture/`，或对应目录的 README |
| 仓库级能力总结 | `reports/overview/` |
| 最终用户交付说明 | `reports/delivery/<case-id>.md` |
| 单个案例某阶段的执行报告 | `reports/<case-id>/<阶段>/REPORT.md`；多次独立报告使用独立文件名或 run-id 子目录，不覆盖旧记录 |
| 阶段检查结果 | `reports/<case-id>/<阶段>/checks/` |
| 阶段运行日志 | `reports/<case-id>/<阶段>/logs/` |
| 仓库级测试日志 | `reports/logs/`，日志默认不提交 |
| 工程维护与整理总结 | `reports/maintenance/` |
| 可追溯的物理轨迹、快照、审阅包 | `cases/<case-id>/<阶段>/` 下既有 run/review 结构 |
| 新回归测试 | `tests/physics/`、`tests/navigation/` 或 `tests/workflow/` |

## 这次为何没有搬动所有目录

- `cases/` 已按案例、六阶段、run-id 和 review-id 分层，包含绑定路径及哈希的不可变证据；不为美观改写它们。
- `archive/` 已分出配置、registry、reports、runs 和 scripts，作为历史快照保持原样。
- 阶段 YAML 的原路径出现在构造政策校验和最终审阅包里；搬动会影响旧门禁，因此保持原路径，不加重复副本或兼容软链接。
- `src/lastmile/` 与 `workflow/` 已区分物理后端和编排层；本次通过职责索引说明模块分组，不同时重构 Python 导入路径。
- `.git/`、`.agents/`、`.codex/`、`.aws/` 是版本控制或工具目录，不归入业务整理范围。

旧报告路径见 [迁移清单](directory-migration.json)。被搬动的日志及 JSON 内容保持不变；Markdown 只修正路径引用，历史结论不变。
