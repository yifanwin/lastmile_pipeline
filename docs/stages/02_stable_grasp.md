# 02 A 点稳定抓法

输入：步骤 1 不变的物理模型与快照；源 Cup_30 资产 grasp；冻结动作和验收协议。

A 是初始成功站位，不是后续构造的失败起点。默认左臂、h=0.738、row 794；不在失败后改行号兜底。规划实际场景 collision mesh，真实缓慢调整躯干，然后执行预抓取、接近、闭合、抬升和保持。

两层证据分别报告：

1. 从旧校准迁入的 5 名义 / 20 扰动轨迹，重算动作前缀和严格 Pick 验收，校验协议、选择规则、扰动预注册，不信报告 success 字段。
2. 当前恢复卡的物理模型上新执行 1 次 A 点 witness，完整动态轨迹与结果独立保存。

输出：historical_revalidation.json、fresh_A_witness/、stable_grasps.json、summary.json。

`--provisional` 允许在人工审阅前运行，只能得到 provisional_verified。人工批准后状态变 verified。未批准场景不能进入步骤 3。Case 2 把手抓法尚未实现为此工作流公共入口；已有后端部件语义不会回退普通抓法。
