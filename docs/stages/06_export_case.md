# 步骤 6：最终人工确认与正式导出

已实现完整流程：不可变最终审阅包 → 人工决定 → 正式导出门禁 → 白名单证据包 → 哈希验证与保存状态回放。

## 当前决定

用户明确选择“3 次成功即可”，因此 `configs/06_export_case.yaml` 记录本次完整链路从方案默认 5 次覆盖为 3 次全成功。其他物理阈值、资产名义 5/5 与小扰动 19/20 门槛不变。实际已有名义 5/5、小扰动 20/20 的复核历史证据，不冒充新试验。

用户随后明确回复“接受（已观看并认可上述各项）”，已记录到当前 `final_v2/accepted_review.json`，actor 为 human、reviewer 为 chat_user、来源为 explicit_user_chat_reply。不是 Agent 自动批准，也不虚构姓名。

## 命令与人工介入

```bash
# 首次生成，已有 review-id 不可覆盖
MUJOCO_GL=disable lastmile_pipeline/bin/lastmile prepare-final --review-id final_v2 --closed-loop-run-id astar_v2
# 本机交互审阅，远程可 ssh -L 18767:127.0.0.1:18767 <server>
MUJOCO_GL=disable lastmile_pipeline/bin/lastmile serve-final-review --review-id final_v2 --port 18767
# 人工也可填写 reviews/final_v2/human_review.json 或下载决定 JSON 后导入
MUJOCO_GL=disable lastmile_pipeline/bin/lastmile accept-final --review-id final_v2 --file /path/to/human_review.json
MUJOCO_GL=disable lastmile_pipeline/bin/lastmile check-final-gate --review-id final_v2
# 审阅前可组装草稿（绝不标记人工接受或数据集准入）
MUJOCO_GL=disable lastmile_pipeline/bin/lastmile export-case --review-id final_v2 --export-id final_v2 --draft
# 只有有效 accepted 人工决定才能正式导出
MUJOCO_GL=disable lastmile_pipeline/bin/lastmile export-case --review-id final_v2 --export-id final_v2
MUJOCO_GL=disable lastmile_pipeline/bin/lastmile verify-export --bundle lastmile_pipeline/cases/case1-cup-001/06_export_case/exports/final_v2
```

上述当前运行已经执行，不应覆盖或重复导入不可变决定。新配置、新证据或新的人工决定必须使用新的 review-id / export-id。旧 `final_v1` 是组装前版本；已由补齐站位图哈希绑定的 `final_v2` 接续，旧版本不放行。

接受必须确认：已观看视频、起点范围证书、仅改机器人初始 base、同侧平移分类、三次全成功要求。可选择 accepted / rejected / reconstruct；pending、agent 批准、过期审阅、文件哈希改变或物理失败一律阻断正式导出。

## 包结构与复现边界

```text
06_export_case/
  reviews/<review-id>/     审阅包、物理资格、可交互 HTML、不可变人工决定
  staging/<export-id>/     未人工接受的草稿：dataset_admitted=false
  exports/<export-id>/     正式接受包：case.json、package_manifest.json
  exports/<export-id>.zip  正式压缩包与外部 receipt 哈希锚点
```

包内包含：来源与修改清单、原始/派生快照、完整编译模型、动作协议、grasp 来源、原始粗站位图（含规划与物理结果）、三次完整真实导航/调整/Pick 记录、接触与支撑判定、保存状态、视频、原始审阅材料、实现快照及环境版本。模型按哈希去重，不改原始轨迹；evidence_index.json 映射原工作区路径到包内路径。只复制明确证据白名单，不复制 `.env`、凭据或 Git 元数据。

分类固定为 **Case 1-R，同侧平移可解决**，不声称必须换边或整条西侧无解。起点不可达依据保守接触界，不伪造失败伸臂动作。

```bash
# 解压后：标准库核验全部文件 SHA-256
python replay_saved_states.py --verify-only
# 需要 MuJoCo/NumPy，直接加载包内编译模型，无需 NAS 资产
MUJOCO_GL=disable python replay_saved_states.py --trial 0
```

这检查/回放保存的真实状态，不冒充重新动力学仿真。重新运行 cuRobo 生成任务仍需要原工程只读资产、GPU 和原依赖；包不声称包含全部外部资产。审阅时的实现快照与当时物理执行文件哈希分别保留，导出实现另记录，不能把新版本代码冒充原执行版本。
