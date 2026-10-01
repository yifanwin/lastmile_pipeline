# 步骤 4 执行与结果总结

**人工选择和原样筛选已完成；当前 episode 14 被筛除，未构造出 Case 1 失败起点。步骤 5 未放行。**

## 执行过程

1. 按方案第六节的首选方式，记录用户明确选择“仅筛选原始起点，不改场景”。该记录不等于人工接受最终 Case。
2. 验证步骤 1–3 的前置门禁、站位图证据审计与哈希，读取原始底盘位姿。
3. 重新验收同一原始站位的躯干调整前缀及严格 Pick 轨迹。
4. 生成原样筛选结果、筛除清单和自包含审阅卡；绑定来源和证据，测试步骤 5 拒绝放行。

## 结果与依据

原始底盘 `[7.183716297, 2.709005594, -1.702443123]` 就是已有成功 A 点。`T00_A_left_h0.7380`（左臂、h=.738、grasp 794）重新验收为 `verified_success / strict_pick`，足以反驳“原始起点无法操作”的候选条件。没有用其他站位失败代替原始起点失败。

- [人工方式记录](../../../cases/case1-cup-001/04_construct_start/human_choice.json)
- [筛选结果与证据哈希](../../../cases/case1-cup-001/04_construct_start/summary.json) · [筛除清单](../../../cases/case1-cup-001/04_construct_start/screening_registry.json)
- [原始 Pick 轨迹](../../../cases/case1-cup-001/03_station_map/runs/coarse_v1/trials/T00_A_left_h0.7380/grasp_0794_trace.json)
- [离线审阅卡](../../../cases/case1-cup-001/04_construct_start/construction_review.html)

底盘初始位姿、目标、家具、资产均未改动。新增物理试验 **0 次**：本步骤复用已有物理证据并重新验收，不冒充新 GPU 实跑。没有生成失败起点配置或导航请求。

## 验证及边界

**67 项单元测试全部通过**；交付链接、决策包哈希及嵌入原图一致性已检查，见 [交付校验](checks/step4-delivery-validation.json)。门禁执行见 [测试日志](../../logs/unit-tests.log)、[步骤 4 门禁日志](logs/step4-gate.log)。门禁退出 1 是预期筛除结果。审阅卡的原始视角使用步骤 1 图像，离线嵌入；没有声称完成浏览器截图验收。

当前仅实现原样筛选分支，未实现扩大新来源的批量筛选，也未完成步骤 5–6、绕行视频或最终 Case 1 闭环。下一步应筛选其他未经修改的原始 episode，并对新来源完成审计及步骤 1–3；保留 episode 14 作为成功正对照。不得未经人工重新选择就自动移动机器人。
