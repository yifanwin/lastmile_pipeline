"""站位图证据导出：不将规划可行或高斯插值当作物理验收。"""
import json,base64,html
from collections import Counter
import numpy as np
from .core import *
from .station_geometry import gaussian_scores

def export_map(case_id,run_id='coarse_v1'):
 import matplotlib;matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 from matplotlib.lines import Line2D
 from matplotlib.patches import Rectangle
 from matplotlib.colors import ListedColormap
 from matplotlib import font_manager
 font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams.update({'font.family':'Noto Sans CJK JP','axes.unicode_minus':False,'font.size':10})
 folder=case_dir(case_id)/STAGES[2]/'runs'/run_id;rows=read(folder/'candidates.json');plans=read(folder/'planning.json');physical=read(folder/'physics.json');frozen=read(folder/'frozen_inputs.json');config=frozen['config'];facts=read(case_dir(case_id)/STAGES[0]/'scene_facts.json');grid=np.load(folder/'floor_grid.npz');xs,ys,floors=grid['xs'],grid['ys'],grid['floors'];xx,yy=np.meshgrid(xs,ys,indexing='ij');bounds=read(folder/'reach_bound.json')
 center=np.array(facts['support_AABB_center']);size=np.array(facts['support_AABB_size']);target=np.array(facts['target_initial_pose'][:2]);A=np.array(facts['initial_base']);pdata={};tdata={}
 for p in plans:pdata.setdefault(p['pose_id'],[]).append(p)
 for p in physical:tdata.setdefault(p['pose_id'],[]).append(p)
 for row in rows:
  pp=pdata.get(row['pose_id'],[]);tt=tdata.get(row['pose_id'],[]);full=[p for p in pp if p['status']=='full_path_planned'];success=[t for t in tt if t['binary_pick_label']==1];failed=[t for t in tt if t['binary_pick_label']==0]
  row.update(planning_queries=len(pp),ik_solutions=sum(bool(p['ik_solved']) for p in pp),full_path_solutions=len(full),planning_status='full_path_planned' if full else 'search_exhausted' if pp else 'not_run',physics_trials=len(tt),physics_successes=len(success),physics_failures=len(failed),physics_status='verified_success' if success else 'executed_failure' if failed else 'attempt_incomplete' if tt else 'not_run',best_planning=full[0] if full else None)
 write(folder/'stations.json',rows)
 # XY 聚合明确采用存在成功优先，保留每个 yaw 的明细；并非平均成功率。
 locations={}
 for r in rows:locations.setdefault(tuple(np.round(r['base'][:2],6)),[]).append(r)
 def glyph(rr):
  if any(r['physics_successes'] for r in rr):return ('#16866b','o','真实 Pick 通过（单次）')
  if any(r['physics_failures'] for r in rr):return ('#c93444','X','完整执行 / 严格 Pick 失败')
  if any(r['physics_status']=='attempt_incomplete' for r in rr):return ('#aa3377','P','物理尝试未完成 Pick')
  if any(r['full_path_solutions'] for r in rr):return ('#287bb5','s','三段路径规划通过（未验收）')
  if any(r['geometry_status']=='eligible' for r in rr):return ('#d99719','o','预算内未找到方案')
  if all(r['geometry_status']=='outside_contact_bound' for r in rr):return ('#7056a7','^','超出保守接触范围界')
  return ('#8a8e94','x','地面 / 初始碰撞几何过滤')
 fig=plt.figure(figsize=(16,10));gs=fig.add_gridspec(2,3,width_ratios=[1.3,1.3,1],height_ratios=[1,1],wspace=.28,hspace=.38);ax=fig.add_subplot(gs[:,0:2]);heat=fig.add_subplot(gs[0,2]);bar=fig.add_subplot(gs[1,2])
 projection_path=folder/'scene_projection.json'
 if projection_path.exists():projection=read(projection_path)
 else:
  import mujoco
  from molmo_spaces.utils.mj_model_and_data_utils import geom_aabb
  m=mujoco.MjModel.from_binary_path(str(folder/'engine/compiled_model.mjb'));d=mujoco.MjData(m);state=np.load(folder/'engine/initial_snapshot.npz')['state'];mujoco.mj_setState(m,d,state,mujoco.mjtState.mjSTATE_INTEGRATION);mujoco.mj_forward(m,d);projection=[];table_id=m.body(facts['support_parent']).id
  for g in range(m.ngeom):
   body=int(m.geom_bodyid[g]);name=m.geom(g).name
   if m.body(body).name.startswith('robot_0/') or int(m.body_rootid[body])==table_id or 'floor' in name.lower() or not (m.geom_contype[g] or m.geom_conaffinity[g]):continue
   c,z=geom_aabb(m,d,[g]);lo=c-z/2;hi=c+z/2
   if hi[2]<.025 or lo[2]>1.85 or np.any(hi[:2]<[xs[0],ys[0]]) or np.any(lo[:2]>[xs[-1],ys[-1]]):continue
   projection.append({'name':name,'min':lo.tolist(),'max':hi.tolist()})
  write(projection_path,projection)
 def backdrop(axis):
  axis.imshow(floors.T,origin='lower',extent=[xs[0],xs[-1],ys[0],ys[-1]],cmap=ListedColormap(['#edf0f3','#fbfcfd']),vmin=0,vmax=1,alpha=.9)
  for obstacle in projection:
   lo=np.array(obstacle['min']);hi=np.array(obstacle['max']);axis.add_patch(Rectangle(lo[:2],*(hi-lo)[:2],facecolor='#c3c7cc',edgecolor='#b4b9c0',alpha=.22,lw=.3,zorder=1.5))
  axis.add_patch(Rectangle(center[:2]-size[:2]/2,*size[:2],facecolor='#eed7b2',edgecolor='#987d54',lw=1.5,zorder=2));axis.scatter(*target,marker='*',s=180,color='#e87521',edgecolor='white',zorder=8)
  axis.set(xlabel='世界 x（m）',ylabel='世界 y（m）');axis.set_aspect('equal');axis.set_xlim(xs[0],xs[-1]);axis.set_ylim(ys[0],ys[-1]);axis.grid(alpha=.15)
 backdrop(ax)
 from matplotlib.patches import Circle
 ax.add_patch(Circle(target,bounds['contact_radius_m'],fill=False,ls='--',color='#7056a7',alpha=.55,lw=1.3,zorder=2))
 for xy,rr in locations.items():
  color,marker,label=glyph(rr);ax.scatter(*xy,color=color,marker=marker,s=90 if marker=='o' and color=='#16866b' else 65,linewidths=1.7,zorder=5)
  if any(r['physics_successes'] for r in rr) and any(r['physics_failures'] for r in rr):ax.scatter(*xy,facecolors='none',edgecolors='#c93444',marker='o',s=165,linewidths=1.8,zorder=6)
  # 有实跑时标出真实采用的 yaw；否则显示该 XY 第一个几何合格朝向。
  tr=next((t for t in physical if np.linalg.norm(np.array(t['base'][:2])-xy)<1e-5),None)
  yaw=tr['base'][2] if tr else next((r['base'][2] for r in rr if r['geometry_status']=='eligible'),rr[0]['base'][2]);ax.arrow(*xy,.11*np.cos(yaw),.11*np.sin(yaw),width=.003,head_width=.04,color=color,alpha=.75,zorder=4)
  if tr:ax.annotate(tr['trial_id'].split('_')[0]+('/A' if tr['pose_id']=='A' else ''),xy,xytext=(5,5),textcoords='offset points',fontsize=8,color='#333',zorder=9)
 ax.text(.02,.98,'箭头：实跑朝向或该点首个采样朝向\n同一 XY 合并：任一采样 yaw 成功即标绿\n灰色环境轮廓仅供参考（AABB）\n空白不是失败；不代表导航可通行',transform=ax.transAxes,va='top',fontsize=10,bbox={'facecolor':'white','alpha':.93,'edgecolor':'#ddd'})
 ax.set_title('Case 1 · 粗站位图：物理结果优先，规划与几何分层',fontsize=15,pad=14)
 legend=[]
 for color,marker,label in [('#16866b','o','真实 Pick 通过（单次）'),('#c93444','X','完整执行 / 严格 Pick 失败'),('#aa3377','P','物理尝试未完成 Pick'),('#287bb5','s','三段路径规划通过（未验收）'),('#d99719','o','预算内未找到方案'),('#7056a7','^','超出保守接触范围界'),('#8a8e94','x','地面 / 初始碰撞几何过滤')]:legend.append(Line2D([0],[0],marker=marker,color='none',markerfacecolor=color,markeredgecolor=color,markersize=8,label=label))
 legend.append(Line2D([0],[0],marker='o',color='none',markerfacecolor='#16866b',markeredgecolor='#c93444',markeredgewidth=1.8,markersize=9,label='同一 XY：另有配置实跑失败（红圈）'))
 legend.append(Line2D([0],[0],ls='--',color='#7056a7',label=f'保守接触界 {bounds["contact_radius_m"]:.3f} m'))
 ax.legend(handles=legend,loc='lower left',fontsize=8.5,framealpha=.97)
 valid=[t for t in physical if t['binary_pick_label'] is not None];point_groups={}
 for t in valid:point_groups.setdefault(tuple(np.round(t['base'][:2],6)),[]).append(t['binary_pick_label'])
 points=np.array(list(point_groups)).reshape(-1,2);labels=np.array([np.mean(v) for v in point_groups.values()]);scores,count=gaussian_scores(points,labels,np.c_[xx.ravel(),yy.ravel()],k=config['interpolation']['knn'],sigma=config['interpolation']['sigma_m'],max_distance=config['interpolation']['max_support_distance_m']);score=scores.reshape(xx.shape);score[~floors]=np.nan
 # 仅作保守显示遮罩：不在桌面 footprint / 环境 AABB 内扩散标签；仍不是导航自由空间。
 display_mask=floors.copy();display_mask&=~((xx>=center[0]-size[0]/2)&(xx<=center[0]+size[0]/2)&(yy>=center[1]-size[1]/2)&(yy<=center[1]+size[1]/2))
 for obstacle in projection:
  lo=np.array(obstacle['min']);hi=np.array(obstacle['max']);display_mask&=~((xx>=lo[0])&(xx<=hi[0])&(yy>=lo[1])&(yy<=hi[1]))
 score[~display_mask]=np.nan
 np.savez_compressed(folder/'gaussian_preview.npz',xs=xs,ys=ys,score=score,neighbor_count=count.reshape(xx.shape),display_mask=display_mask,physical_points=points,physical_labels=labels)
 backdrop(heat);im=heat.imshow(score.T,origin='lower',extent=[xs[0],xs[-1],ys[0],ys[-1]],cmap='YlGnBu',vmin=0,vmax=1,alpha=.7,zorder=3)
 if len(points):heat.scatter(points[:,0],points[:,1],c=labels,vmin=0,vmax=1,cmap='YlGnBu',edgecolors='#333',s=35,zorder=5)
 heat.set_title('Gaussian 预览：仅真实完整 Pick 标签\n≤0.30 m 局部支持；不是成功概率',fontsize=10);fig.colorbar(im,ax=heat,fraction=.045,pad=.03,label='局部标签插值分数')
 if len(set(labels.tolist()))<2:heat.text(.02,.02,'标签无正负变化：不能据此定位可达边界',transform=heat.transAxes,fontsize=8,bbox={'facecolor':'white','alpha':.92,'edgecolor':'none'})
 totals={'候选 base pose':len(rows),'几何合格':sum(r['geometry_status']=='eligible' for r in rows),'至少一组三段规划':sum(r['full_path_solutions']>0 for r in rows),'新物理试验':len(physical),'真实 Pick 通过':sum(t['binary_pick_label']==1 for t in physical),'完整 Pick 失败':sum(t['binary_pick_label']==0 for t in physical)}
 bar.barh(list(totals),list(totals.values()),color=['#89919b','#677b8e','#287bb5','#aa3377','#16866b','#c93444']);bar.invert_yaxis();bar.set_xlabel('数量（前 3 项按含 yaw 的 base pose）');bar.grid(axis='x',alpha=.2)
 for i,v in enumerate(totals.values()):bar.text(v+.5,i,str(v),va='center',fontsize=9)
 bar.set_xlim(0,max(totals.values())*1.2);bar.set_title('覆盖与证据强度',fontsize=11)
 fig.suptitle('episode 14 / house 103 / Cup_30 · 目标与场景不变 · 仅放置机器人底盘进行独立操作标注',fontsize=14,y=.97);fig.text(.05,.018,'五个躯干状态 × 左右臂 × grasp 794；稀疏单次物理标签不等于区域稳定性，也不证明任何绕行路线。',fontsize=10,color='#444')
 for ext in ['png','svg']:fig.savefig(folder/f'station_map.{ext}',dpi=170,bbox_inches='tight')
 plt.close(fig)
 # 详尽规划矩阵与 pose/yaw 明细，保留失败状态而不是强行 0/1 化。
 import csv
 with (folder/'station_table.csv').open('w') as f:
  fields=['pose_id','side','x','y','yaw_rad','geometry_status','planning_queries','ik_solutions','full_path_solutions','physics_trials','physics_successes','physics_failures','physics_status'];w=csv.DictWriter(f,fieldnames=fields);w.writeheader()
  for r in rows:w.writerow({**{key:r[key] for key in fields if key in r},'x':r['base'][0],'y':r['base'][1],'yaw_rad':r['base'][2]})
 # 与 MoMa-Kitchen 的 sparse-floor 对应：每条真实二值样本匹配最近 floor cell，未知不填 0。
 from scipy.spatial import cKDTree
 floor_xy=np.c_[xx[floors],yy[floors]];floor_tree=cKDTree(floor_xy)
 sparse=[];threshold=float(np.sqrt(2)*config['sampling']['floor_grid_spacing_m']/2+.001)
 for t in valid:
  distance,index=floor_tree.query(np.array(t['base'][:2]));point=floor_xy[index]
  sparse.append({'trial_id':t['trial_id'],'base_x':t['base'][0],'base_y':t['base'][1],'base_yaw_rad':t['base'][2],'arm':t['side'],'h':t['h'],'binary_pick_label':t['binary_pick_label'],'floor_point_index':int(index),'floor_x':float(point[0]),'floor_y':float(point[1]),'nearest_distance_m':float(distance),'mapped':bool(distance<=threshold)})
 with (folder/'floor_sparse_samples.csv').open('w') as f:
  keys=['trial_id','base_x','base_y','base_yaw_rad','arm','h','binary_pick_label','floor_point_index','floor_x','floor_y','nearest_distance_m','mapped'];writer=csv.DictWriter(f,fieldnames=keys);writer.writeheader();writer.writerows(sparse)
 write(folder/'floor_sparse_metadata.json',{'coordinate_frame':'world xy','nearest_threshold_m':threshold,'threshold_method':'half floor-grid diagonal + 1 mm; not paper-specific theta','floor_point_count':len(floor_xy),'actual_binary_samples':len(sparse),'mapped_samples':sum(x['mapped'] for x in sparse),'unknown_points_not_zero':True,'no_camera_frame_dataset_export':True})
 summary={'case_id':case_id,'run_id':run_id,'status':'coarse_map_complete','source_index':14,'approved_sides':read(case_dir(case_id)/STAGES[0]/'human_review.json')['suitable_sides'],'candidate_poses':len(rows),'candidate_xy_locations':len(locations),'geometry_counts':dict(Counter(r['geometry_status'] for r in rows)),'planning_queries':len(plans),'planning_counts':dict(Counter(p['status'] for p in plans)),'poses_with_full_path':sum(r['full_path_solutions']>0 for r in rows),'physical_trials':len(physical),'physical_pick_successes':sum(t['binary_pick_label']==1 for t in physical),'physical_complete_pick_failures':sum(t['binary_pick_label']==0 for t in physical),'physical_incomplete_attempts':sum(t['binary_pick_label'] is None for t in physical),'gaussian_uses_only_complete_physics':True,'gaussian_not_probability':True,'gaussian_display_mask':'floor minus support footprint and projected environment AABBs; conservative visual mask, not navigation GT','A_control_success':any(t['pose_id']=='A' and t['binary_pick_label']==1 for t in physical),'scene_objects_unchanged':True,'base_placement_is_not_navigation':True,'grasp_ids':[794],'arms':config['planning']['arms'],'heights':config['planning']['heights'],'contact_radius_bound_m':bounds['contact_radius_m'],'scope':'coarse finite sampling and search budget; only executed poses have physical labels; no whole-side unreachable claim from search failure','completed_at_utc':now()};write(folder/'summary.json',summary)
 # self-contained 页面：图嵌入、逐 trial 原因与原始证据链接。
 enc=base64.b64encode((folder/'station_map.png').read_bytes()).decode();tr=''
 for t in physical:
  esc=html.escape;tr+=f'<tr><td>{esc(t["trial_id"])}</td><td>{esc(str([round(x,4) for x in t["base"]]))}</td><td>{esc(t["side"])}/{t["h"]:.4f}</td><td>{esc(str(t["revalidated_verdict"]))}</td><td>{esc(str(t["binary_pick_label"]))}</td><td><a href="{esc(t["result_path"])}">原结果</a></td></tr>'
 doc=f'''<!doctype html><html lang="zh"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Case 1 粗站位图</title><style>body{{font:16px/1.6 system-ui;background:#f6f8fa;color:#223;max-width:1450px;margin:24px auto;padding:20px}}section{{background:white;padding:20px;border-radius:12px;margin:20px 0}}img{{width:100%;height:auto}}table{{border-collapse:collapse;font-size:13px;width:100%}}th,td{{padding:9px;border-bottom:1px solid #ddd;text-align:left}}pre{{white-space:pre-wrap}}.warn{{border-left:5px solid #c68c23}}a{{color:#276fac}}</style><h1>{html.escape(case_id)} · 步骤 3 粗站位图</h1><section class="warn"><b>绿色只有真实严格 Pick 通过；蓝色只是三段规划通过。</b><br>冻结场景，每次先恢复完整快照，再只设置待标注的机器人底盘位姿。放置是操作 oracle 的试验初始化，不是导航轨迹。Gaussian 是局部标签插值，不能作为验收或成功概率。</section><section><img src="data:image/png;base64,{enc}" alt="粗站位图"><p><a href="station_map.svg">SVG</a> · <a href="station_map.png">PNG</a> · <a href="station_table.csv">含 yaw 的全部站位 CSV</a> · <a href="stations.json">逐点证据 JSON</a> · <a href="floor_sparse_samples.csv">真实二值标签→地面点对应</a> · <a href="planning.json">双臂五高度规划明细</a></p></section><section><h2>物理试验结果</h2><table><thead><tr><th>Trial</th><th>base [x,y,yaw]</th><th>臂 / h</th><th>严格结果 / 原因</th><th>完整 Pick 标签</th><th>证据</th></tr></thead><tbody>{tr}</tbody></table><p>None 表示未完成动态 Pick；不能当不可达或插值中的失败标签。各点单次试验不估计成功率。</p></section><section><h2>覆盖摘要</h2><pre>{html.escape(json.dumps(summary,ensure_ascii=False,indent=2))}</pre></section><section><h2>下一步人工介入</h2><p>根据真实成功点和边界检查结果选择步骤 4 的构造方式。当前没有验证绕行或构造最终困难起点。南侧范围界、几何阻挡和预算内无解必须分别读，不互相替代。</p></section></html>'''
 (folder/'station_map.html').write_text(doc)
 (folder/'station_map.py').write_text("#!/usr/bin/env python3\n# 原始证据上的可复现站位图入口；绘图实现见 src/lastmile/workflow/station_plot.py\nfrom pathlib import Path\nimport subprocess\np=Path(__file__).resolve();W=p.parents[5]\nsubprocess.check_call([str(W/'bin/lastmile'),'plot-station-map','--case-id',p.parents[3].name,'--run-id',p.parent.name])\n")
 stage=case_dir(case_id)/STAGES[2];write(stage/'latest.json',{'run_id':run_id,'summary':str((folder/'summary.json').relative_to(stage)),'map_png':str((folder/'station_map.png').relative_to(stage)),'map_html':str((folder/'station_map.html').relative_to(stage))});return summary
