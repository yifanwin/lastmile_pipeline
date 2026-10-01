# 仓库协作指南

## 语言与工作方式

- 默认使用中文沟通、编写说明和总结；代码标识符、命令及现有协议字段保持原名。
- 修改前先阅读 `README.md`、对应的 `docs/stages/` 文档及相关代码、测试。
- 保持改动聚焦，不顺手重排或格式化无关代码。保留用户已有的未提交修改。
- 完成较复杂的任务后，先用 Markdown 总结执行过程、结果及验证情况；明确未运行的测试和限制。

## 项目定位与目录

本项目是 Lastmile 半自动场景构造和连续导航抓取验收管线，按六阶段组织。自动系统生成证据，人工决定是否保留场景，人工决定不能覆盖物理失败。

- `bin/lastmile`：唯一推荐的命令入口，加载相邻 `molmospaces` 的环境。
- `src/lastmile/workflow/`：阶段编排、人工门禁、审阅服务、证据审计和导出。
- `src/lastmile/`：MuJoCo / cuRobo、导航、动作协议和物理验收后端。
- `configs/`：阶段 YAML、动作协议及 agent 配置。
- `tests/`：基于 `unittest` 的回归测试，按 `physics/`、`navigation/`、`workflow/` 分组；子目录保留 `__init__.py` 以支持递归发现。
- `cases/<case-id>/`：来源输入、`manifest.json` 和六阶段证据；自动结果与人工决定分开记录。
- `docs/stages/`：各阶段输入、输出与通过条件。
- `reports/`：`delivery/` 存交付说明，`overview/` 存实现总结，`<case-id>/<阶段>/` 存阶段报告、`checks/` 检查记录和 `logs/` 日志。
- `archive/calibration_v1/`：只读历史证据，不作为新实现入口。

阶段顺序为恢复审阅、稳定抓取、站位图、困难起点构造、连续导航抓取、最终人工接受与导出。当前进度查看 `TODO.md` 和对应 case 的 manifest，不把本指南当作动态状态记录。

## 运行与测试

以下命令均从本仓库根目录执行：

```bash
# 查看 CLI 和 case 状态
./bin/lastmile --help
./bin/lastmile status --case-id case1-cup-001

# 单元测试，复用相邻工程的虚拟环境
PYTHONPATH=src ../molmospaces/.venv/bin/python -m unittest discover -s tests -v

# 定向回归示例
PYTHONPATH=src ../molmospaces/.venv/bin/python -m unittest discover -s tests -p 'test_workflow.py' -v
```

- 入口默认 `CUDA_VISIBLE_DEVICES=5`、`MUJOCO_GL=egl`；按可用设备显式覆盖，不假定所有环境都有 GPU 5。
- 依赖相邻工程的 `molmospaces/setup_env.sh`、虚拟环境和资产。缺失时报告依赖问题，不用替代模型伪造通过结果。
- 修改门禁、协议、导航或证据处理逻辑时，补充对应回归测试，尤其覆盖失败、过期或篡改输入。
- 单元测试不等于真实仿真验收；涉及物理成功的结论必须有本次真实执行证据。
- 耗时仿真或生成新 case 前确认运行范围；不要为普通代码验证重跑已交付 case 或覆盖已有 run。

## 核心约束

- 不修改原工程、benchmark、源 XML、原始资产和历史轨迹。历史路径通过 workflow 映射，不改原始记录。
- 新来源先审计；不猜资产名，不将当前特定 episode 的稳定证据推广到其他来源。
- 人工批准只能来自明确的人工记录。agent 只有建议权，不得代填批准、绕过门禁或执行其生成的代码、坐标。
- `check-gate` 仅检查条件，不代表执行下一阶段；预校准成功也不自动取得人工批准。
- 区分真实抓取成功、规划通过、几何过滤、未知区域及搜索预算耗尽；未知标签不能填成失败零值。
- 保持严格物理验收：双指真实受力接触、至少抬升 5 cm、无支撑持续至少 2 s，并检查基座、躯干、头和闲置臂约束；具体阈值以协议及阶段配置为准，不为通过测试放宽。
- 连续导航到站后抓取不得用重置替代闭环；原始成功起点不能充当困难失败起点。
- `robot_only` 分支只改机器人 base，目标和家具保持不变；其他场景修改需要明确授权、来源、修改清单及验收。

## 证据、配置与安全

- 新试验使用独立 `run-id`；续跑只允许复用相同冻结配置，不覆盖既有审阅包和历史记录。
- 修改证据格式、manifest、协议或配置时，同步检查摘要校验、失效门禁、阶段文档和测试。
- JSON 写入优先复用 `workflow/core.py` 的原子写入与摘要工具，保留来源和审计链。
- `.env` 仅用于读取 `LLM_API_KEY`、`LLM_BASE_URL`、`LLM_MODEL`；不复制、打印或提交凭据。外发图像或场景事实须有适用于本次操作的授权。
- 审阅服务仅监听本机；服务 API 不调用 agent。
- 大型模型、快照、轨迹、视频和导出包已有 `.gitignore` 规则，不强制加入版本控制；必要小型回执及交付文档按现有约定维护。
- 新报告和日志放到对应分类，不重新堆放在 `reports/` 根目录。目录职责见 `docs/architecture/directory-layout.md`。
- 最终交付突出文档及必要图片、视频，工程审计材料保留但不要求用户阅读完整 SHA 检查材料。
