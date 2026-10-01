#!/usr/bin/env python3
"""确定性 FK 证据图；不以肩高或模型误差推断抓取成功率。"""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
W=Path(__file__).absolute().parents[1]
r=json.loads((W/'reports/model_calibration.json').read_text())
rows=[p for p in r['fk_samples'] if p['side']=='left']
fig,ax=plt.subplots(figsize=(6.5,3.5))
ax.plot([p['h'] for p in rows],[p['shoulder_xyz'][2] for p in rows], 'o-',color='#3498db')
ax.set(xlabel='Torso parameter h (rad; not meters)',ylabel='Shoulder z in MJCF world (m)',title='RBY1 coupled torso: measured forward kinematics')
ax.grid(alpha=.2);fig.tight_layout()
folder=W/'reports/figures';folder.mkdir(exist_ok=True)
for ext in ['svg','png']:fig.savefig(folder/f'torso_fk.{ext}',dpi=160)
plt.close(fig)
fig,ax=plt.subplots(figsize=(6.5,3.5))
x=np.arange(len(r['fk_samples']))
raw=[np.linalg.norm(p['position_delta_m'])*1000 for p in r['fk_samples']]
corrected=[p['base_frame_position_error_m']*1000 for p in r['fk_samples']]
ax.plot(x,raw,'o-',label='Incorrect z=0 world frame',color='#e74c3c')
ax.plot(x,corrected,'o-',label='Measured base frame (5 mm offset removed)',color='#27ae60')
ax.set(xlabel='5 torso states x 2 arms',ylabel='TCP translation error (mm)',title='Frame mismatch must not become a reachability label')
ax.legend(fontsize=8);ax.grid(alpha=.2);fig.tight_layout()
for ext in ['svg','png']:fig.savefig(folder/f'frame_alignment.{ext}',dpi=160)
plt.close(fig)
