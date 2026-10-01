#!/usr/bin/env python3
from pathlib import Path
from lastmile.audit import extract
W = Path(__file__).absolute().parents[1]
rows, assets = extract(W.parent/'molmospaces_data/assets', W)
print(f'候选 {len(rows)}，资产 {len(assets)}，正式白名单 0（待物理验收）')
for r in rows:
    print(r['source_index'], r['asset_id'], r['support_category'])
