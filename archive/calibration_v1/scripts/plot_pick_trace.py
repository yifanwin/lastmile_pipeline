#!/usr/bin/env python3
"""真实物理轨迹，不把计划 TCP 当作目标提升。"""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from lastmile.audit import write_json
W=Path(__file__).absolute().parents[1]
p=W/'runs/repeats/E009-cup-force-aware/nominal_00.json'
e=json.loads(p.read_text());rows=e['samples'];initial=e['initial'];t=np.array([r['time_s'] for r in rows]);t-=t[0]
lift=np.array([r['height_m']-initial['height_m'] for r in rows]);base=np.array([r['base'] for r in rows]);b0=np.array(initial['base'])
translation=np.linalg.norm(base[:,:2]-b0[:2],axis=1);yaw=np.abs(np.arctan2(np.sin(base[:,2]-b0[2]),np.cos(base[:,2]-b0[2])))
q=np.array([r['torso'] for r in rows]);h=np.array([r['h'] for r in rows]);torso=np.max(np.abs(q-np.c_[0*h,h,-2*h,h,0*h,0*h]),axis=1)
fingers=np.array([len(r['selected_fingers']) for r in rows]);support=np.array([r['support_contact'] for r in rows]);held=(fingers==2)&(~support)&(lift>=.05)
start=None
for i,ok in enumerate(held):
 if not ok:start=None
 elif start is None:start=i
hold=t[-1]-t[start] if start is not None else 0
metrics={'source_trace':str(p),'lift_final_m':float(lift[-1]),'lift_max_m':float(lift.max()),'base_translation_max_m':float(translation.max()),'base_yaw_max_deg':float(np.rad2deg(yaw.max())),'torso_command_error_max_rad':float(torso.max()),'final_continuous_bilateral_no_support_lift_s':float(hold),'illegal_sample_count':sum(bool(r['illegal']) for r in rows),'physics_samples':len(rows),'sample_dt_s':.004,'note':'hold diagnostic does not replace full relative-slip and protocol validator'}
write_json(W/'reports/cup_trace_metrics.json',metrics)
fig,ax=plt.subplots(3,1,figsize=(10,7),sharex=True)
ax[0].plot(t,lift*100,label='actual target lift');ax[0].axhline(5,ls='--',c='r',label='5 cm threshold');ax[0].set_ylabel('Lift (cm)');ax[0].legend(loc='upper left')
ax[1].plot(t,fingers,label='selected fingers touching target');ax[1].plot(t,support.astype(int),label='support contact',alpha=.6);ax[1].set_ylabel('Contact count / flag');ax[1].legend(loc='upper left')
ax[2].plot(t,translation*1000,label='base translation (mm)');ax[2].plot(t,torso*1000,label='torso error (mrad)');ax[2].axhline(2,ls='--',c='r',label='both limits = 2');ax[2].set_ylabel('mm / mrad');ax[2].set_xlabel('Physical time since pregrasp (s)');ax[2].legend(loc='upper left')
for a in ax:
 a.grid(alpha=.2)
 if start is not None:a.axvspan(t[start],t[-1],color='green',alpha=.12)
fig.suptitle('E009: force-aware fresh physics, Cup_30 / left / h=0.738 / grasp row 794')
fig.tight_layout()
for ext in ['png','svg']:fig.savefig(W/f'reports/figures/cup_physical_trace.{ext}',dpi=160)
print(json.dumps(metrics,indent=2))
