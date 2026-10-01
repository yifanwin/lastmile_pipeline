"""步骤 1：真实恢复、相机校准、审阅卡和待人工确认记录。"""
import copy,json,html
from pathlib import Path
import numpy as np
import mujoco
from PIL import Image,ImageDraw,ImageFont
from ..simulation import Simulation,body_pose
from ..audit import sha256
from molmo_spaces.utils.mj_model_and_data_utils import body_aabb
from .core import WORKSPACE,LEGACY,case_dir,read,write,digest,load_config,engine_context,update,STAGES,now

def physical_signature(model):
 fields=['body_parentid','body_pos','body_quat','body_mass','body_inertia','body_gravcomp','jnt_type','jnt_pos','jnt_axis','jnt_range','dof_damping','geom_type','geom_bodyid','geom_pos','geom_quat','geom_size','geom_contype','geom_conaffinity','geom_friction','geom_solref','geom_solimp','mesh_vert','mesh_face','actuator_trnid','actuator_gainprm','actuator_biasprm','actuator_ctrlrange']
 import hashlib
 h=hashlib.sha256()
 for key in fields:h.update(key.encode());h.update(np.asarray(getattr(model,key)).tobytes())
 h.update(str(model.opt.timestep).encode());h.update(np.asarray(model.opt.gravity).tobytes())
 return h.hexdigest()

def import_source(index):
 # 已校准源配置保留只读；未登记索引明确停止，先执行 source audit，不猜资产 ID。
 path=LEGACY/f'registry/episodes/{index:04d}.json'
 if not path.exists():raise ValueError('此索引尚未审计；请先登记源场景/资产，禁止从名字猜测')
 episode=read(path);asset=read(LEGACY/f'registry/assets/{episode["provenance"]["asset_id"]}.json')
 if digest(episode['provenance']['scene_xml'])!=episode['provenance']['scene_sha256']:raise ValueError('源场景 XML 已改变，拒绝复用旧恢复缓存')
 if digest(asset['asset_xml'])!=asset['asset_xml_sha256'] or digest(asset['grasp_path'])!=asset['grasp_sha256']:raise ValueError('源资产/grasp 已改变，拒绝复用证据')
 return episode,asset

def restore_cameras(sim):
 m=sim.model;rows=[]
 for spec in sim.episode['cameras']:
  name='robot_0/'+spec['name'];camera=m.camera(name).id;expected=m.body(spec['reference_body_names'][0]).id
  if int(m.cam_bodyid[camera])!=expected:raise ValueError('相机绑定身体不匹配：'+name)
  m.cam_pos[camera]=spec['camera_offset'];q=np.array(spec['camera_quaternion']);m.cam_quat[camera]=q/np.linalg.norm(q);m.cam_fovy[camera]=spec['fov'];m.cam_mode[camera]=mujoco.mjtCamLight.mjCAMLIGHT_FIXED
  rows.append({'name':name,'reference_body':m.body(expected).name,'position':m.cam_pos[camera].tolist(),'quaternion':m.cam_quat[camera].tolist(),'fov':float(m.cam_fovy[camera]),'source':'benchmark robot_mounted camera'})
 mujoco.mj_forward(m,sim.data);return rows

def render_views(sim,folder,center,size):
 views=folder/'views';views.mkdir();m,d=sim.model,sim.data;m.vis.global_.offwidth=960;m.vis.global_.offheight=720
 renderer=mujoco.Renderer(m,height=720,width=960);font=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',21)
 target_geoms=[g for g in range(m.ngeom) if int(m.body_rootid[m.geom_bodyid[g]])==sim.target_root]
 def shot(camera,name,caption):
  renderer.update_scene(d,camera=camera);rgb=renderer.render().copy();renderer.enable_segmentation_rendering();renderer.update_scene(d,camera=camera);seg=renderer.render().copy();renderer.disable_segmentation_rendering()
  mask=(seg[:,:,1]==mujoco.mjtObj.mjOBJ_GEOM)&np.isin(seg[:,:,0],target_geoms);image=Image.fromarray(rgb);draw=ImageDraw.Draw(image);draw.rectangle([0,0,959,35],fill=(20,30,40));draw.text((12,3),caption,font=font,fill='white')
  if mask.any():
   y,x=np.where(mask);box=[max(0,int(x.min())-7),max(36,int(y.min())-7),min(959,int(x.max())+7),min(719,int(y.max())+7)];draw.rectangle(box,outline=(255,166,55),width=3)
  image.save(views/name);return int(mask.sum())
 camera=mujoco.MjvCamera();camera.lookat[:]=center;camera.lookat[2]=.45;camera.distance=4.7;camera.azimuth=0;camera.elevation=-89
 counts={'overhead':shot(camera,'overhead.png','完整恢复俯视图 · 橙框为目标实例')}
 counts['initial_robot_view']=shot('robot_0/head_camera','initial_robot_view.png','机器人初始视角 · 按 benchmark 恢复 head_camera')
 camera.lookat[:]=d.xpos[sim.target];camera.distance=.65;camera.azimuth=155;camera.elevation=-35
 counts['closeup']=shot(camera,'target_closeup.png','目标近景 · 仅用于审阅，不代表抓取通过')
 renderer.close()
 # 桌面边界/桌边建议是几何事实和提示，不能包装为站位图或已验证路线。
 import matplotlib;matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 fig,ax=plt.subplots(figsize=(7,6));ax.add_patch(plt.Rectangle(center[:2]-size[:2]/2,*size[:2],facecolor='#f0dbbb',edgecolor='#555',lw=2))
 xy=d.xpos[sim.target,:2];base=sim.group('base');ax.scatter(*xy,c='#e67e22',marker='*',s=150,label='target');ax.scatter(*base[:2],c='#267eb4',s=70,label='initial success candidate A')
 ax.arrow(*base[:2],.23*np.cos(base[2]),.23*np.sin(base[2]),width=.01,color='#267eb4')
 for label,offset in [('north',[0,size[1]/2+.45]),('south',[0,-size[1]/2-.45]),('east',[size[0]/2+.45,0]),('west',[-size[0]/2-.45,0])]:
  p=center[:2]+offset;ax.text(*p,label,ha='center',va='center',color='#666')
 ax.set(xlim=(center[0]-1.5,center[0]+1.5),ylim=(center[1]-1.65,center[1]+1.65),xlabel='world x (m)',ylabel='world y (m)',title='Support footprint and side hints (navigation NOT verified)');ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend();fig.tight_layout()
 for ext in ['png','svg']:fig.savefig(views/f'support_access_map.{ext}',dpi=150)
 plt.close(fig);return counts

def render_card(case_id):
 folder=case_dir(case_id)/STAGES[0];facts=read(folder/'scene_facts.json');packet_sha=digest(folder/'review_packet.json');agent=read(folder/'agent_review.json') if (folder/'agent_review.json').exists() else {'status':'not_requested'};human=read(folder/'human_review.json') if (folder/'human_review.json').exists() else {'decision':'pending'}
 esc=lambda x:html.escape(str(x));stage2=case_dir(case_id)/STAGES[1];stable=read(stage2/'summary.json') if (stage2/'summary.json').exists() else {'status':'not_run'}
 cards=''.join(f'<figure><img src="views/{name}"><figcaption>{title}</figcaption></figure>' for name,title in [('overhead.png','俯视：源场景全部保留，橙框定位目标'),('initial_robot_view.png','初始视角：benchmark 相机和头部关节恢复'),('target_closeup.png','目标近景：检查目标身份、支撑与邻近杂物'),('support_access_map.png','支撑边界与桌边提示：未验证导航/绕行')])
 doc='''<!doctype html><html lang="zh"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>场景人工审阅</title><style>body{font:16px/1.65 system-ui;margin:0;background:#f4f6f8;color:#223;max-width:1300px;margin:auto;padding:24px}h1{font-size:28px}.notice,section{background:white;padding:22px;border-radius:10px;margin:18px 0}.notice{border-left:5px solid #d18a27}.grid{display:grid;grid-template-columns:1fr 1fr;gap:18px}figure{margin:0;background:white;padding:12px;border-radius:8px}img{width:100%;height:auto}figcaption{font-size:14px;color:#556}pre{white-space:pre-wrap;overflow-wrap:anywhere}label{display:block;margin:10px 0}input,select,textarea,button{font:inherit}textarea{width:95%;height:80px}button{padding:8px 14px;margin:8px;cursor:pointer}.checks label{display:inline-block;margin-right:15px}@media(max-width:800px){.grid{grid-template-columns:1fr}}</style>'''
 doc+=f'<h1>{esc(case_id)} · 步骤 1 场景审阅</h1><div class="notice"><b>人工状态：{esc(human.get("decision"))}</b>。agent 只给建议；目标正确、确为桌面、可操作桌边与是否保留 Case 1，必须由你确认。步骤 2 可以预校准，但未获批准不会进入步骤 3。</div><section><b>来源</b>：episode {facts["source_index"]} / house {facts["house_index"]}<br><b>目标</b>：{esc(facts["asset_id"])} / {esc(facts["target_name"])}<br><b>支撑</b>：{esc(facts["support_parent"])}<br><b>A 点</b>：{esc(facts["initial_base"])}（成功候选，不是困难起点）<br><b>步骤 2</b>：{esc(stable.get("status"))}</section><div class="grid">{cards}</div>'
 doc+=f'<section><h2>agent 场景建议（不能覆盖物理结果）</h2><pre>{esc(json.dumps(agent,ensure_ascii=False,indent=2))}</pre></section><section><h2>人工决定</h2><label>审阅者 <input id="reviewer" placeholder="你的名字"></label><label><input type="checkbox" id="target_correct"> 目标实例正确</label><label><input type="checkbox" id="is_tabletop"> 确实是桌面支撑</label><div class="checks">可操作桌边：'+''.join(f'<label><input type="checkbox" name="side" value="{side}">{side}</label>' for side in ['north','south','east','west'])+'</div><label>保留类型 <select id="type"><option value="case1_distance">Case 1 距离型</option><option value="reject">不保留/换场景</option></select></label><label>说明<textarea id="notes" placeholder="指出窄通道、可绕行边、希望调整的位置等"></textarea></label><button onclick="submitReview(\'approved\')">批准</button><button onclick="submitReview(\'rejected\')">拒绝/换场景</button><button onclick="downloadReview()">下载人工确认 JSON（离线）</button><pre id="result"></pre></section>'
 safe_case=json.dumps(case_id);safe_sha=json.dumps(packet_sha)
 doc+=f'''<script>function data(decision){{return {{case_id:{safe_case},review_packet_sha256:{safe_sha},decision,reviewer:document.getElementById('reviewer').value,target_correct:document.getElementById('target_correct').checked,is_tabletop:document.getElementById('is_tabletop').checked,suitable_sides:[...document.querySelectorAll('input[name=side]:checked')].map(x=>x.value),retain_case_type:document.getElementById('type').value,notes:document.getElementById('notes').value}}}}async function submitReview(decision){{try{{if(location.protocol==='file:')throw new Error('离线打开请下载 JSON，再用 lastmile approve 导入；在线请启动 serve-review');let r=await fetch('/api/review',{{method:'POST',headers:{{'Content-Type':'application/json'}},body:JSON.stringify(data(decision))}});let x=await r.json();document.getElementById('result').textContent=JSON.stringify(x,null,2);if(!r.ok)throw new Error(x.error)}}catch(e){{document.getElementById('result').textContent=e.message}}}}function downloadReview(){{let d=data(document.getElementById('type').value==='reject'?'rejected':'approved');let u=URL.createObjectURL(new Blob([JSON.stringify(d,null,2)],{{type:'application/json'}}));let a=document.createElement('a');a.href=u;a.download={safe_case}+'-human-review.json';a.click();URL.revokeObjectURL(u)}} </script></html>'''
 (folder/'review.html').write_text(doc)
 (folder/'review_card.md').write_text(f'# {case_id} 场景审阅卡\n\n人工状态：{human.get("decision")}。请打开 [交互审阅页](review.html)。\n\n目标：{facts["asset_id"]}；source_index：{facts["source_index"]}；A 点：`{facts["initial_base"]}`。\n\n'+''.join(f'![{name}](views/{name})\n\n' for name in ['overhead.png','initial_robot_view.png','target_closeup.png','support_access_map.png'])+'可接近边/绕行只是审阅建议，不代表真实导航通过。\n')

def run_restore(case_id,index=14,cache=None):
 folder=case_dir(case_id)/STAGES[0]
 if (folder/'review_packet.json').exists():raise ValueError('已恢复的审阅包不可覆盖；新配置请用新的 case_id')
 episode,asset=import_source(index);case_dir(case_id).mkdir(parents=True,exist_ok=True);write(case_dir(case_id)/'inputs/source_episode.json',episode);write(case_dir(case_id)/'inputs/asset.json',asset)
 context=engine_context(case_id,episode,asset);cache=Path(cache) if cache else WORKSPACE/load_config('01_restore_review.yaml')['model_cache']
 sim=Simulation.from_cached(context,index,cache,folder) if cache.exists() else Simulation(context,index,folder)
 signature=physical_signature(sim.model);camera_rows=restore_cameras(sim)
 if physical_signature(sim.model)!=signature:raise ValueError('相机恢复改变了物理模型')
 # 导出独立 camera-faithful model，而不是把修改写回历史缓存。
 link=folder/'compiled_model.mjb'
 if link.is_symlink():link.unlink()
 mujoco.mj_saveModel(sim.model,str(link),None)
 errors=[]
 for name,pose in episode['scene_modifications']['object_poses'].items():
  actual=sim.data.xpos[sim.model.body(name).id]
  q=sim.data.xquat[sim.model.body(name).id];expected=np.asarray(pose[3:]);expected=expected/np.linalg.norm(expected)
  if np.linalg.norm(actual-np.array(pose[:3]))>1e-5 or 1-abs(float(q@expected))>1e-6:errors.append(name)
 if errors:raise ValueError('场景对象姿态恢复不一致')
 for group,values in episode['robot']['init_qpos'].items():
  actual=sim.q([f'gripper_finger_{group[0]}1',f'gripper_finger_{group[0]}2']) if group.endswith('gripper') else sim.group(group)
  if not np.allclose(actual,values,atol=1e-5):raise ValueError('机器人关节恢复不一致：'+group)
 parent=episode['provenance']['support_parent'];center,size=body_aabb(sim.model,sim.data,sim.model.body(parent).id,visible_only=False)
 counts=render_views(sim,folder,center,size)
 for _ in range(250):mujoco.mj_step(sim.model,sim.data)
 support=sim.sample('left',0.)['support_contact'];settled=float(sim.data.xpos[sim.target,2]);sim.restore()
 facts={'source_index':index,'house_index':episode['house_index'],'asset_id':asset['asset_id'],'target_name':sim.target_name,'support_parent':parent,'support_category':episode['provenance']['support_category'],'initial_base':sim.group('base').tolist(),'initial_robot_groups':episode['robot']['init_qpos'],'target_initial_pose':episode['task']['pickup_obj_start_pose'],'support_AABB_center':center.tolist(),'support_AABB_size':size.tolist(),'all_object_poses_restored':True,'restored_object_count':len(episode['scene_modifications']['object_poses']),'added_object_count':len(episode['scene_modifications']['added_objects']),'cameras':camera_rows,'source_image_resolution':episode['img_resolution'],'visible_target_pixels':counts,'support_diagnostic_after_1s':{'bearing_support_contact':support,'settled_target_z':settled,'not_grasp_success':True},'physical_model_signature_sha256':signature,'camera_only_model_changes':True,'source_scene_sha256':episode['provenance']['scene_sha256'],'benchmark_sha256':episode['provenance']['benchmark_sha256'],'side_hints':'north/south/east/west refer to world xy; no side or detour is navigation-verified','human_approved':False,'created_at_utc':now()}
 write(folder/'scene_facts.json',facts)
 files={'scene_facts.json':digest(folder/'scene_facts.json'),**{str(p.relative_to(folder)):digest(p) for p in sorted((folder/'views').glob('*'))}}
 write(folder/'review_packet.json',{'schema_version':1,'case_id':case_id,'files':files,'compiled_model_sha256':digest(link),'initial_snapshot_sha256':digest(folder/'initial_snapshot.npz'),'source_episode_sha256':digest(case_dir(case_id)/'inputs/source_episode.json')})
 write(folder/'human_review.template.json',{'case_id':case_id,'review_packet_sha256':digest(folder/'review_packet.json'),'decision':'pending','reviewer':'','target_correct':None,'is_tabletop':None,'suitable_sides':[],'retain_case_type':'case1_distance','notes':''})
 update(case_id,STAGES[0],status='awaiting_human_review',restore_complete=True,review_card_complete=True,human_approved=False,review_packet_sha256=digest(folder/'review_packet.json'));render_card(case_id)
 return facts
