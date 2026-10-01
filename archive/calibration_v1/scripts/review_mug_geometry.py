#!/usr/bin/env python3
"""资产局部坐标审阅卡：真实 mesh 顶点与原 grasp TCP，非把手标签。"""
from pathlib import Path
import json
import xml.etree.ElementTree as ET
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
W=Path(__file__).absolute().parents[1]
assets=['Mug_1','Mug_2','RoboTHOR_mug_ai2_1_v']
fig,axes=plt.subplots(2,3,figsize=(12,7))
for col,uid in enumerate(assets):
    record=json.loads((W/f'registry/assets/{uid}.json').read_text())
    xml=Path(record['asset_xml']);root=ET.parse(xml).getroot()
    mesh_files={m.get('name'):xml.parent/m.get('file') for m in root.iter('mesh') if m.get('file')}
    points=[]
    def walk(body,parent):
        local=np.eye(4);local[:3,3]=np.fromstring(body.get('pos','0 0 0'),sep=' ')
        local[:3,:3]=Rotation.from_quat(np.fromstring(body.get('quat','1 0 0 0'),sep=' '),scalar_first=True).as_matrix()
        pose=parent@local
        for geom in body.findall('geom'):
            if geom.get('mesh') not in mesh_files:continue
            p=mesh_files[geom.get('mesh')]
            v=np.array([[float(x) for x in line.split()[1:4]] for line in p.read_text().splitlines() if line.startswith('v ')])
            gp=np.eye(4);gp[:3,3]=np.fromstring(geom.get('pos','0 0 0'),sep=' ')
            gp[:3,:3]=Rotation.from_quat(np.fromstring(geom.get('quat','1 0 0 0'),sep=' '),scalar_first=True).as_matrix()
            points.append((np.c_[v,np.ones(len(v))] @ (pose@gp).T)[:,:3])
        for child in body.findall('body'):walk(child,pose)
    for b in root.find('worldbody').findall('body'):walk(b,np.eye(4))
    v=np.concatenate(points)
    with np.load(record['grasp_path']) as z:g=z['transforms'][:,:3,3].astype(float)
    for row,(x,y) in enumerate([(0,1),(2,1)]):
        ax=axes[row,col]
        ax.scatter(v[:,x],v[:,y],s=.5,c='#555555',alpha=.4,rasterized=True,label='Source mesh vertices')
        ax.scatter(g[:,x],g[:,y],s=3,c='#e67e22',alpha=.15,rasterized=True,label='Droid grasp TCPs')
        ax.set_aspect('equal');ax.set(title=uid if row==0 else 'Not a handle-contact annotation',xlabel=['x','y','z'][x]+' (m)',ylabel='y (m)')
        ax.grid(alpha=.15)
fig.suptitle('Review only: asset-local geometry and 1000 grasp poses per mug',fontsize=13)
fig.tight_layout()
for ext in ['png','svg']:fig.savefig(W/f'reports/figures/mug_geometry_review.{ext}',dpi=160)
