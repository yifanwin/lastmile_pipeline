# 步骤 6 执行总结

**已实现最终审阅、人工确认导入、正式导出和独立验证流程。用户已明确接受 P131→A 的 Case 1-R，并同意完整链路采用 3 / 3 次全成功。**

## 执行过程

1. 记录用户覆盖：完整链路由方案默认 5 次改为本次 3 次全成功，未修改物理阈值或资产稳定性门槛。原三次原始轨迹、5/5 名义与 20/20 扰动历史复核证据均保留。
2. 重新复核原始闭环证据、源快照、派生快照和视频绑定；生成 `final_v2` 不可变审阅包。审阅卡覆盖起点视角、原始站位图、路线、成功视频、重复试验和修改清单。
3. 实现人工 accepted / rejected / reconstruct 导入。pending、agent 批准、跨站请求、旧审阅、哈希变化或物理不通过都不能正式导出。
4. 先组装未准入草稿，确认独立 SHA 检查与保存状态终态回放可运行；随后收到用户明确“接受（已观看并认可上述各项）”，记录为 human / chat_user / explicit_user_chat_reply，不虚构姓名。
5. 正式包只复制白名单证据，包含完整编译模型、快照、协议、grasp 来源、原始站位图、三次完整物理记录、视频与环境版本；原始 JSON 不改写，路径由 evidence_index.json 映射。

## 分类与边界

该 case 是 **Case 1-R，同侧平移可解决**。P131 距目标 1.556593 m，超过双臂、原地朝向和连续合法躯干的保守接触上界 1.217127 m；P131→A 三次连续导航抓取均严格通过。不是“必须换边”，也不声称整条西侧无解。

3 次全成功是用户本次明确门槛，不代表完成默认五次要求或推断总体成功率。先前补充试验在用户中断后留有部分材料、无活动进程；记录为 incomplete，不纳入本次正式准入统计，不改标为成功或失败。

## 交付

- [正式 ZIP（460,601,005 字节，约 439 MiB）](../../../cases/case1-cup-001/06_export_case/exports/final_v2.zip) · [正式包入口](../../../cases/case1-cup-001/06_export_case/exports/final_v2/README.md)
- [case.json](../../../cases/case1-cup-001/06_export_case/exports/final_v2/case.json) · [外部导出回执](../../../cases/case1-cup-001/06_export_case/exports/final_v2-receipt.json)
- [最终审阅卡](../../../cases/case1-cup-001/06_export_case/reviews/final_v2/review.html) · [真实人工接受记录](../../../cases/case1-cup-001/06_export_case/reviews/final_v2/accepted_review.json)
- [三次物理原始证据审计](../../../cases/case1-cup-001/05_closed_loop/runs/astar_v2/evidence_audit.json) · [步骤 6 操作说明](../../../docs/stages/06_export_case.md)

原始视频仍为 45.96 s；导航 8×、躯干调整 4×、抓取 1×，标注加速与仅渲染剖切。完整原始物理时间每次 251.50 s。独立回放加载包内模型，无需 NAS 资产；这是保存状态回放，不冒充重新动力学仿真。重新 cuRobo 生成仍需要原工程依赖与只读资产。

## 验证

100 项单元测试通过，包括纯临时夹具中的人工接受导出路径、草稿不准入、缺确认拒绝、物理失败拒绝、哈希变更拒绝、路径隔离和不可覆盖。HTTP 页面/视频可读，`.env` 和越界路径 404，agent 与跨站批准请求被拒绝，测试没有创建人工接受记录。实际接受来自用户后续明确回复。

正式包 **257 个文件 SHA**、ZIP 全部 **258 个条目 CRC**及归档哈希、独立终态回放均通过；审阅页嵌入图片/本地链接校验见[交付验证记录](checks/step6-delivery-validation.json)。没有声称浏览器截图验收；最终审阅图沿用已实际检查的物理证据图。
