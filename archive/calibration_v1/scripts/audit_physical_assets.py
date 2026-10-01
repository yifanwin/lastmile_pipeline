#!/usr/bin/env python3
"""编译单个原始资产，记录质量/碰撞与局部边界；不是稳定抓取验收。"""
from pathlib import Path
import json
import mujoco
import numpy as np
from lastmile.audit import write_json
W=Path(__file__).absolute().parents[1];rows=[]
for path in sorted((W/'registry/assets').glob('*.json')):
 r=json.loads(path.read_text());m=mujoco.MjModel.from_xml_path(r['asset_xml']);d=mujoco.MjData(m);mujoco.mj_forward(m,d)
 collision=[i for i in range(m.ngeom) if m.geom_contype[i] or m.geom_conaffinity[i]]
 mesh_ids=set(int(m.geom_dataid[i]) for i in collision if m.geom_type[i]==mujoco.mjtGeom.mjGEOM_MESH)
 row={'asset_id':r['asset_id'],'source_asset_sha256':r['asset_xml_sha256'],'total_mass_kg':float(m.body_mass.sum()),'collision_geom_count':len(collision),'collision_types':sorted(set(int(m.geom_type[i]) for i in collision)),'collision_friction':sorted(set(tuple(float(x) for x in m.geom_friction[i]) for i in collision)),'collision_mesh_count':len(mesh_ids),'status':'static_physical_properties_only_not_grasp_admission','caution':'source scene collision decomposition can differ; runtime scene contacts are authoritative'}
 rows.append(row);print(json.dumps(row),flush=True)
write_json(W/'reports/asset_physical_properties.json',{'assets':rows,'note':'no mass/friction/geometry edits; these measurements do not imply handle fit or stable grasp'})
