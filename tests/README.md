# 回归测试

| 分组 | 测试文件 | 范围 |
|---|---|---|
| `physics/` | `test_protocol.py` | 动作、躯干联动、严格 Pick、部件语义 |
| `navigation/` | `test_navigation.py` | A*、穿角、路径简化、导航约束及连续执行参数 |
| `workflow/` | `test_workflow.py` | 人工批准、证据篡改及前置门禁 |
| `workflow/` | `test_station_map.py` | 候选、几何、物理标签及插值 |
| `workflow/` | `test_construction.py` | 原样筛选、构造政策及困难候选门禁 |
| `workflow/` | `test_final_review.py` | 最终决定、重复要求及草稿/正式包 |

从仓库根目录执行全部测试：

```bash
MPLCONFIGDIR=/tmp/lastmile_mpl PYTHONPATH=src \
  ../molmospaces/.venv/bin/python -m unittest discover -s tests -v
```

只检查一个分组或文件：

```bash
PYTHONPATH=src ../molmospaces/.venv/bin/python -m unittest discover -s tests/navigation -v
PYTHONPATH=src ../molmospaces/.venv/bin/python -m unittest discover -s tests -p 'test_workflow.py' -v
```

各分组保留 `__init__.py`，保证 `unittest discover -s tests` 递归发现测试。测试不是新的物理试验，不操作已交付的 case。
