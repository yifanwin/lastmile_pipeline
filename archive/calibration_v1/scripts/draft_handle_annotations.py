#!/usr/bin/env python3
"""仅生成保守区域草案；不得自动 human_confirmed 或自动入白名单。"""
from pathlib import Path
import json
import numpy as np
from lastmile.audit import write_json
W=Path(__file__).absolute().parents[1];template=json.loads((W/'configs/handle_contact_v1.json').read_text())
for uid in ['Mug_1','Mug_2','RoboTHOR_mug_ai2_1_v']:
 r=json.loads((W/f'registry/assets/{uid}.json').read_text())
 # 由几何审阅卡可见把手位于 +x；用狭窄外侧区域远离杯身，边界一律禁用。
 annotation={**template,'asset_id':uid,'asset_xml_sha256':r['asset_xml_sha256'],'grasp_sha256':r['grasp_sha256'],'human_confirmed':False,'status':'draft_requires_review_and_force_contact_validation','local_cup_up_axis':[0.,1.,0.],
  'allowed_regions':[{'min':[.078,-.024,-.010],'max':[.105,.040,.010]}],
  'forbidden_regions':[{'min':[-.10,-.10,-.10],'max':[.073,.10,.10]}],
  'region_basis':'conservative outer +x handle slab from mesh review; connection/rim/boundary excluded; not a confirmed mesh part segmentation',
  'review_figure':'reports/figures/mug_geometry_review.png','allowed_grasp_ids':[],
  'tcp_region_candidate_ids':[]}
 with np.load(r['grasp_path']) as z:tcp=z['transforms'][:,:3,3].astype(float)
 box=annotation['allowed_regions'][0];mask=((tcp>=box['min'])&(tcp<=box['max'])).all(axis=1)
 annotation['tcp_region_candidate_ids']=np.flatnonzero(mask).tolist()
 annotation['note']='TCP region candidates are review hints, not compliant grasp labels; allowed_grasp_ids stays empty until actual finger geometry/contact review.'
 destination=W/f'configs/parts/{uid}.json'
 if destination.exists() and json.loads(destination.read_text()).get('human_confirmed'):raise ValueError('拒绝覆盖人工确认')
 write_json(destination,annotation);print(uid,len(annotation['tcp_region_candidate_ids']))
