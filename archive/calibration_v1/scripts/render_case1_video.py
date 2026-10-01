"""以保存的真实物理 qpos 渲染交付视频；加速段明确标注，墙体仅视觉剖切。"""
import argparse,json,subprocess
from pathlib import Path
import numpy as np
import mujoco
from PIL import Image,ImageDraw,ImageFont
from lastmile.audit import sha256,write_json
p=argparse.ArgumentParser();p.add_argument('--run-id',required=True);p.add_argument('--trial',type=int,default=0);p.add_argument('--preview-only',action='store_true');a=p.parse_args()
W=Path(__file__).absolute().parents[1];root=W/'runs/case1'/a.run_id;dest=W/'deliverables/case1_r_001';dest.mkdir(parents=True,exist_ok=True)
c=json.loads((root/'case_config.json').read_text());trial=root/f'trial_{a.trial:02d}';result=json.loads((trial/'result.json').read_text())
if result['status']!='verified_success':raise ValueError('只交付严格物理验收通过的闭环，不给失败视频贴成功标签')
z=np.load(trial/'video_states.npz');times=z['time_s'];qpos=z['qpos'];phases=z['phase'];m=mujoco.MjModel.from_binary_path(str(root/'compiled_model.mjb'));d=mujoco.MjData(m);target=m.body(c['target_name']).id
m.vis.global_.offwidth=960;m.vis.global_.offheight=720
main=mujoco.Renderer(m,height=720,width=960);small=mujoco.Renderer(m,height=240,width=320)
fontpath='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc';f=lambda size:ImageFont.truetype(fontpath,size)
f14,f18,f23,f28=f(14),f(18),f(23),f(28)
labels={'start':'01 起点：另一侧目标超出接触范围','navigation':'02 真实导航：沿桌西侧绕行','torso_adjust':'03 实测到站：冻结底盘 / 调整躯干','pregrasp':'04 cuRobo 重新规划：预抓取','approach':'05 接近目标','close':'06 双指闭合','lift':'07 抬离桌面','hold':'08 稳定保持'}
speeds={'navigation':8,'torso_adjust':4};indices=[];k=0
while k<len(times):
 indices.append(k);phase=str(phases[k]);stride=speeds.get(phase,1)
 nxt=k+stride
 # 不跳过阶段边界；末尾保证最终保持帧可见。
 while nxt>k+1 and nxt<len(times) and str(phases[nxt])!=phase:nxt-=1
 k=nxt
if indices[-1]!=len(times)-1:indices.append(len(times)-1)
cam=mujoco.MjvCamera();cam.lookat[:]=[7.40,1.57,.58];cam.distance=3.65;cam.azimuth=142;cam.elevation=-56
close=mujoco.MjvCamera();close.distance=.90;close.azimuth=150;close.elevation=-27
initial_z=.8012844308118927
nav=json.loads((trial/'navigation_trace.json').read_text());navpts=np.array([r['base'][:2] for r in nav['samples']]);navtime=np.array([r['time_s'] for r in nav['samples']])
pick=json.loads((trial/'pick/grasp_0794_trace.json').read_text());picktimes=np.array([r['time_s'] for r in pick['samples']]);arrival=np.array(result['arrival']['actual_arrival'])
certificate=json.loads((W/'reports/case1_reach_certificate.json').read_text())
held_secs=[];holding_start=None;relative_ref=None
for row in pick['samples']:
 held=len(row['selected_fingers'])==2 and not row['support_contact'] and row['height_m']-initial_z>=.05
 pose=np.array(row['relative_pose'])
 if not held:holding_start=None;relative_ref=None
 elif holding_start is None:holding_start=row['time_s'];relative_ref=pose
 if held and relative_ref is not None:
  angle=np.arccos(np.clip((np.trace(relative_ref[:3,:3].T@pose[:3,:3])-1)/2,-1,1))
  if np.linalg.norm(pose[:3,3]-relative_ref[:3,3])>.01 or angle>np.deg2rad(5):holding_start=None;relative_ref=None
 held_secs.append(row['time_s']-holding_start if holding_start is not None else 0.)

def hide_walls(renderer):
 for j in range(renderer.scene.ngeom):
  g=renderer.scene.geoms[j]
  if g.objtype==mujoco.mjtObj.mjOBJ_GEOM and 0<=g.objid<m.ngeom:
   name=m.geom(g.objid).name.lower()
   if any(x in name for x in ['wall_','window_','ceiling']):g.rgba[3]=0.
 # 只影响绘制，绝不影响物理接触。
 renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=False

def frame(i):
 d.qpos[:]=qpos[i];d.time=times[i];mujoco.mj_forward(m,d);phase=str(phases[i]);speed=speeds.get(phase,1)
 main.update_scene(d,camera=cam);hide_walls(main);picture=Image.new('RGB',(1280,720),(17,25,34));picture.paste(Image.fromarray(main.render()),(0,0))
 close.lookat[:]=d.xpos[target];small.update_scene(d,camera=close);hide_walls(small);picture.paste(Image.fromarray(small.render()),(960,62))
 draw=ImageDraw.Draw(picture);draw.rectangle([0,0,959,70],fill=(17,25,34));draw.text((22,8),'Case 1-R  |  真实绕桌导航 → 固定底盘抓取',font=f28,fill='white');draw.text((22,43),labels.get(phase,phase),font=f18,fill=(111,221,242))
 draw.text((974,12),'目标近景 · 同一物理时刻',font=f18,fill='white');draw.text((974,38),'Cup_30 / 普通抓法 / 左臂',font=f14,fill=(173,186,201))
 draw.rectangle([0,664,959,719],fill=(17,25,34));draw.text((22,672),f'物理时间 {times[i]:6.2f} s   |   本段 {speed}× 播放   |   仅墙体视觉剖切，碰撞全部保留',font=f18,fill='white')
 lift=float(d.xpos[target,2]-initial_z);b=np.array([d.qpos[m.joint('robot_0/'+n).qposadr[0]] for n in ['base_x','base_y','base_theta']]);bd=float(np.linalg.norm(b[:2]-arrival[:2]))*1000
 draw.text((978,312),f'目标抬升  {lift*100:5.1f} cm',font=f23,fill=(111,221,242));finger_count=0;support=True
 if times[i]>=picktimes[0]:
  j=min(len(picktimes)-1,int(np.searchsorted(picktimes,times[i],side='right')-1));r=pick['samples'][j];finger_count=len(r['selected_fingers']);support=r['support_contact']
 draw.text((978,352),f'承力手指  {finger_count} / 2',font=f18,fill='white');draw.text((978,380),f'桌面支撑  {"有" if support else "无"}',font=f18,fill='white')
 if phase in ['pregrasp','approach','close','lift','hold']:draw.text((978,410),f'抓取底盘漂移  {bd:.3f} mm',font=f18,fill='white')
 else:draw.text((978,410),'导航用真实底盘位置反馈',font=f18,fill='white')
 # 俯视示意：轨迹只使用当前时刻之前的真实底盘位置，不显示未走路线为实测。
 region=(979,459,1264,666);draw.rectangle(region,outline=(87,105,123),width=1)
 lo=np.array([6.40,.15]);hi=np.array([8.48,3.02]);scale=min((region[2]-region[0])/(hi[0]-lo[0]),(region[3]-region[1])/(hi[1]-lo[1]))
 def screen(v):return (region[0]+float((v[0]-lo[0])*scale),region[3]-float((v[1]-lo[1])*scale))
 tc=np.array(c['table_center']);ts=np.array(c['table_size']);a1=screen(tc[:2]-ts[:2]/2);b1=screen(tc[:2]+ts[:2]/2);draw.rectangle([a1[0],b1[1],b1[0],a1[1]],fill=(71,57,43),outline=(165,132,81))
 pts=navpts[navtime<=times[i]]
 if len(pts)>1:draw.line([screen(v) for v in pts[::10]],fill=(83,194,230),width=3)
 for v,color in [(c['start_base'],(234,96,89)),(c['goal_base'],(111,211,140)),(d.xpos[target],(251,190,70))]:
  x,y=screen(v);draw.ellipse([x-4,y-4,x+4,y+4],fill=color)
 x,y=screen(b);draw.ellipse([x-6,y-6,x+6,y+6],fill='white');draw.line([(x,y),(x+14*np.cos(b[2]),y-14*np.sin(b[2]))],fill='white',width=2)
 draw.text((974,670),'蓝：已行驶  红：起点  绿：终点',font=f14,fill=(173,186,201))
 if phase=='start':
  draw.rectangle([18,96,676,175],fill=(17,25,34));draw.text((30,104),f'起点距目标 {certificate["start_base_to_target_xy_m"]:.2f} m > 保守接触上界 {certificate["base_to_target_contact_radius_bound_m"]:.2f} m',font=f23,fill=(255,191,95));draw.text((30,140),'覆盖双臂、原地朝向及连续合法躯干范围；不能原地抓取',font=f18,fill='white')
 if phase=='hold':
  duration=held_secs[j]
  draw.rectangle([18,96,553,154],fill=(17,25,34));draw.text((30,106),f'双指承力 / 无支撑 / 稳定保持 {duration:.2f} s',font=f23,fill=(111,211,140) if duration>=2 else (255,191,95))
 return picture
if a.preview_only:
 for name,t in [('start',0),('navigation',40),('torso',float(result['navigation_end_time_s'])+15),('hold',times[-1])]:
  i=min(len(times)-1,int(np.searchsorted(times,t)));frame(i).save(dest/f'preview_{name}.png')
else:
 output=dest/'case1_r_001.mp4'
 if output.exists():raise ValueError('不覆盖已交付视频；请先保留/改名')
 process=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-vcodec','rawvideo','-pix_fmt','rgb24','-s','1280x720','-r','25','-i','-','-an','-c:v','libx264','-preset','medium','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(output)],stdin=subprocess.PIPE)
 for n,i in enumerate(indices):
  process.stdin.write(np.asarray(frame(i)).tobytes())
  if n%100==0:print('frames',n,'/',len(indices),flush=True)
 process.stdin.close();code=process.wait()
 if code:raise RuntimeError('ffmpeg failed')
 write_json(dest/'video_manifest.json',{'case_id':c['case_id'],'run_id':a.run_id,'trial':a.trial,'video_sha256':sha256(output),'states_sha256':sha256(trial/'video_states.npz'),'case_config_sha256':sha256(root/'case_config.json'),'physics_duration_s':float(times[-1]),'frames':len(indices),'fps':25,'video_duration_s':len(indices)/25,'render_only_changes':['walls/windows transparent in render scene; collision unchanged','Chinese overlays and current-time trajectory inset'],'playback_speed':{'navigation':8,'torso_adjust':4,'start_and_pick':1},'not_generated_or_interpolated_motion':True})
 print(output,flush=True)
main.close();small.close()
