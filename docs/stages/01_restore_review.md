# 01 自动恢复 / 人工确认

输入：已审计 episode、对应资产、源 XML / grasp 的 SHA256、全状态恢复缓存。恢复房屋、额外资产和所有物体位姿、机器人初始关节、benchmark 相机。模型仅校正相机参数，物理签名不变。源不匹配必须报错，不能跳过缺失物体。

输出：独立 compiled_model.mjb、initial_snapshot.npz、scene_facts.json、四视图、review_packet.json、review.html、human_review.template.json。橙框来自目标实例分割，不是手工猜位置。桌边图只作几何提示，不作为可绕行证明。审阅渲染为 960×720，源采集分辨率另行记录。

人工字段：审阅者、目标是否正确、是否桌面、适合操作桌边、是否保留 Case 1 距离型、备注。使用网页批准/拒绝，或下载 JSON 后 CLI 导入。审批绑定审阅包哈希，证据改变则失效。agent API 可给建议，不能批准。

完成定义：自动恢复和卡片完成不等于人工批准；无绕行可能则换场景。
