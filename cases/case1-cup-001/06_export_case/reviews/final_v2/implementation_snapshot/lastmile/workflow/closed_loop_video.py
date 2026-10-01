"""只使用验收成功试验的真实保存状态渲染视频，不生成或插值运动。"""
import json,subprocess,html,base64
from pathlib import Path
import numpy as np
from . import core as c
from .construction import checked_file

def render_closed_loop(case_id,run_id):
    import mujoco
    from PIL import Image,ImageDraw,ImageFont
    c.case_dir(run_id);case=c.case_dir(case_id);root=case/c.STAGES[4]/'runs'/run_id;summary=c.read(root/'summary.json');packet=c.read(root/'evidence_manifest.json')
    for name,sha in packet['files'].items():checked_file(root,name,sha)
    if not summary['closed_loop_verified']:raise ValueError('失败试验不能渲染为成功交付')
    audit=c.read(root/'evidence_audit.json')
    if not audit['all_checks_passed'] or audit['strict_successes_recomputed']!=summary['strict_successes']:raise ValueError('闭环原始证据尚未审计')
    c.review_packet_valid(case_id)
    if c.digest(root/'engine/compiled_model.mjb')!=c.read(case/c.STAGES[0]/'review_packet.json')['compiled_model_sha256']:raise ValueError('视频模型与冻结源模型不一致')
    config=c.read(root/'frozen_inputs.json')['config'];construction=case/c.STAGES[3]/'runs'/summary['construction_run_id'];certificate=c.read(construction/'reach_certificate.json');trial=root/'trial_00';result=c.read(trial/'result.json')
    folder=root/'delivery'
    if (folder/'case1_r_closed_loop.mp4').exists():raise ValueError('不覆盖交付视频')
    folder.mkdir(exist_ok=True);states=np.load(trial/'video_states.npz');times=states['time_s'];qpos=states['qpos'];phases=states['phase'];m=mujoco.MjModel.from_binary_path(str(root/'engine/compiled_model.mjb'));d=mujoco.MjData(m);target=m.body(config['target_name']).id
    m.vis.global_.offwidth=960;m.vis.global_.offheight=720;main=mujoco.Renderer(m,height=720,width=960);small=mujoco.Renderer(m,height=240,width=320)
    font='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc';f16=ImageFont.truetype(font,16);f19=ImageFont.truetype(font,19);f25=ImageFont.truetype(font,25)
    speed={'navigation':8,'torso_adjust':4};labels={'start':'01 困难起点 P131：双臂接触范围不足','navigation':'02 A* 路线 · 底盘真实伺服 · 拐点停车','torso_adjust':'03 保留实测到站状态 · 动态调整躯干','pregrasp':'04 cuRobo 预抓取','approach':'05 接近目标','close':'06 双指闭合','lift':'07 抬离桌面','hold':'08 无支撑稳定保持'}
    indices=[];k=0
    while k<len(times):
        indices.append(k);phase=str(phases[k]);nxt=k+speed.get(phase,1)
        while nxt>k+1 and nxt<len(times) and str(phases[nxt])!=phase:nxt-=1
        k=nxt
    if indices[-1]!=len(times)-1:indices.append(len(times)-1)
    camera=mujoco.MjvCamera();camera.lookat[:]=[7.35,1.65,.65];camera.distance=3.8;camera.azimuth=145;camera.elevation=-55
    close=mujoco.MjvCamera();close.distance=.85;close.azimuth=150;close.elevation=-30
    nav=c.read(trial/'navigation_trace.json')['samples'];navxy=np.array([r['base'][:2] for r in nav]);navt=np.array([r['time_s'] for r in nav]);pick=c.read(trial/'pick/grasp_0794_trace.json');pickt=np.array([r['time_s'] for r in pick['samples']]);z0=pick['initial']['height_m'];arrival=np.array(result['actual_arrival']['actual_arrival']);held=[];begin=None;reference=None
    from ..validation import rotation_distance
    for row in pick['samples']:
        valid=len(row['selected_fingers'])==2 and not row['support_contact'] and row['height_m']-z0>=.05;pose=np.array(row['relative_pose'])
        if not valid:begin=None;reference=None
        elif begin is None:begin=row['time_s'];reference=pose
        if valid and reference is not None and (np.linalg.norm(pose[:3,3]-reference[:3,3])>.01 or rotation_distance(reference[:3,:3],pose[:3,:3])>np.deg2rad(5)):begin=None;reference=None
        held.append(row['time_s']-begin if begin is not None else 0.)
    def render_scene(renderer,cam):
        renderer.update_scene(d,camera=cam)
        for j in range(renderer.scene.ngeom):
            g=renderer.scene.geoms[j]
            if g.objtype==mujoco.mjtObj.mjOBJ_GEOM and 0<=g.objid<m.ngeom and any(v in (m.geom(g.objid).name or '').lower() for v in ['wall_','window_','ceiling']):g.rgba[3]=0.
        renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=False
        return Image.fromarray(renderer.render())
    def frame(i):
        d.qpos[:]=qpos[i];d.time=times[i];mujoco.mj_forward(m,d);phase=str(phases[i]);picture=Image.new('RGB',(1280,720),(17,25,34));picture.paste(render_scene(main,camera),(0,0));close.lookat[:]=d.xpos[target];picture.paste(render_scene(small,close),(960,68));draw=ImageDraw.Draw(picture)
        draw.rectangle([0,0,959,74],fill=(17,25,34));draw.text((18,7),'Case 1-R | 困难起点 → A* 导航 → cuRobo 抓取',font=f25,fill='white');draw.text((18,44),labels.get(phase,phase),font=f19,fill=(111,221,242));draw.text((972,13),'目标近景 · 同一物理时刻',font=f19,fill='white');draw.text((972,42),'Cup_30 / 左臂 / grasp 794',font=f16,fill=(180,193,210))
        draw.rectangle([0,666,959,719],fill=(17,25,34));draw.text((18,674),f'物理时间 {times[i]:.2f} s | 本段 {speed.get(phase,1)}× | 墙体仅视觉剖切，物理碰撞保留',font=f19,fill='white')
        draw.text((973,322),f'目标抬升 {100*(d.xpos[target,2]-z0):5.1f} cm',font=f25,fill=(111,221,242));b=np.array([d.qpos[m.joint('robot_0/'+n).qposadr[0]] for n in ['base_x','base_y','base_theta']]);j=None
        if times[i]>=pickt[0]:j=min(len(pickt)-1,int(np.searchsorted(pickt,times[i],side='right')-1));row=pick['samples'][j]
        draw.text((973,362),f'承力手指 {len(row["selected_fingers"]) if j is not None else 0} / 2',font=f19,fill='white');draw.text((973,390),f'桌面支撑 {"无" if j is not None and not row["support_contact"] else "有"}',font=f19,fill='white');draw.text((973,420),'到站无重置 · 实测状态规划',font=f19,fill='white')
        region=(973,460,1267,664);draw.rectangle(region,outline=(80,100,120),width=1);lo=np.array([6.25,.15]);hi=np.array([8.5,3.15]);scale=min((region[2]-region[0])/(hi[0]-lo[0]),(region[3]-region[1])/(hi[1]-lo[1]))
        def screen(v):return(region[0]+float((v[0]-lo[0])*scale),region[3]-float((v[1]-lo[1])*scale))
        center=np.array(config['table_center']);size=np.array(config['table_size']);a=screen(center[:2]-size[:2]/2);e=screen(center[:2]+size[:2]/2);draw.rectangle([a[0],e[1],e[0],a[1]],fill=(70,57,43),outline=(164,132,81));pts=navxy[navt<=times[i]]
        if len(pts)>1:draw.line([screen(v) for v in pts[::10]],fill=(83,194,230),width=3)
        for v,color in [(config['start_base'],(234,96,89)),(config['goal_base'],(111,211,140)),(d.xpos[target],(251,190,70))]:
            x,y=screen(v);draw.ellipse([x-4,y-4,x+4,y+4],fill=color)
        x,y=screen(b);draw.ellipse([x-5,y-5,x+5,y+5],fill='white');draw.line([(x,y),(x+13*np.cos(b[2]),y-13*np.sin(b[2]))],fill='white',width=2);draw.text((973,675),'蓝：已走轨迹  红：P131  绿：A',font=f16,fill=(180,193,210))
        if phase=='start':
            draw.rectangle([18,97,805,168],fill=(17,25,34));draw.text((30,103),f'起点距离 {certificate["start_base_to_target_xy_m"]:.3f} m > 接触上界 {certificate["contact_radius_m"]:.3f} m',font=f25,fill=(255,191,95));draw.text((30,139),'覆盖双臂 / 原地任意朝向 / 连续合法躯干；几何证书，不是伪造失败动作',font=f16,fill='white')
        if phase=='hold' and j is not None:
            draw.rectangle([18,97,575,157],fill=(17,25,34));draw.text((30,107),f'双指承力 / 无支撑 / 稳定保持 {held[j]:.2f} s',font=f25,fill=(111,211,140) if held[j]>=2 else (255,191,95))
        return picture
    previews={}
    for phase in ['start','navigation','lift','hold']:
        found=np.flatnonzero(phases==phase)
        if len(found):i=int(found[-1] if phase=='hold' else found[len(found)//2]);name=f'preview_{phase}.png';frame(i).save(folder/name);previews[phase]=name
    output=folder/'case1_r_closed_loop.mp4';process=subprocess.Popen(['ffmpeg','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s','1280x720','-r','25','-i','-','-an','-c:v','libx264','-preset','medium','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(output)],stdin=subprocess.PIPE)
    try:
        for n,i in enumerate(indices):
            process.stdin.write(np.asarray(frame(i)).tobytes())
            if n%100==0:print('rendered',n,'/',len(indices),flush=True)
    finally:process.stdin.close()
    if process.wait():raise RuntimeError('ffmpeg failed')
    main.close();small.close()
    manifest={'case_id':case_id,'run_id':run_id,'trial':0,'video_sha256':c.digest(output),'states_sha256':c.digest(trial/'video_states.npz'),'physical_evidence_manifest_sha256':c.digest(root/'evidence_manifest.json'),'physics_duration_s':float(times[-1]),'video_duration_s':len(indices)/25,'frames':len(indices),'fps':25,'playback_speed':{'navigation':8,'torso_adjust':4,'start_and_pick':1},'render_only_changes':['walls/windows/ceiling visually transparent; physical model untouched','overlays and measured trajectory inset'],'only_saved_physical_states_no_motion_interpolation':True,'strict_successes':summary['strict_successes'],'human_final_accepted':False,'previews':previews}
    c.write(folder/'video_manifest.json',manifest)
    poster=base64.b64encode((folder/'preview_hold.png').read_bytes()).decode()
    graph=base64.b64encode((folder/'closed_loop_evidence.png').read_bytes()).decode() if (folder/'closed_loop_evidence.png').exists() else None
    graph_section=f'<section><h2>规划路线与实测轨迹 / 抬升曲线</h2><img src="data:image/png;base64,{graph}" alt="真实路线和抬升曲线"><p>虚线是 A* 参考，彩色线是三次实测轨迹。抬升曲线不能单独代表严格抓取通过。</p></section>' if graph else ''
    page=f'''<!doctype html><html lang="zh"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Case 1-R 证据审阅</title><style>body{{font:17px/1.7 system-ui;max-width:1200px;margin:30px auto;padding:20px;background:#f5f7f9}}video,img{{width:100%}}section{{padding:22px;background:white;border-radius:12px;margin:18px 0}}code{{overflow-wrap:anywhere}}</style><h1>Case 1-R · P131 → A · A* + cuRobo</h1><section><b>{summary['strict_successes']} / {summary['trials_requested']} 次新连续试验严格通过；等待人工最终确认。</b><p>起点来自步骤 3 站位图。仅改变机器人初始底盘，目标和家具不变。起点距离 {certificate['start_base_to_target_xy_m']:.3f} m 超出双臂接触上界 {certificate['contact_radius_m']:.3f} m；可看到目标。不是声称“整条西侧都不能操作”。</p></section><section><video controls preload="metadata" poster="data:image/png;base64,{poster}"><source src="case1_r_closed_loop.mp4" type="video/mp4"></video><p>视频是 trial 0 的真实保存状态回放。导航 8×、躯干调整 4×、抓取 1×，加速已标注；墙体仅视觉剖切。导航到站不重置，再以实测状态重新规划抓取。</p></section>{graph_section}<section><h2>证据</h2><p><a href="../summary.json">三次结果</a> · <a href="video_manifest.json">视频溯源</a> · <a href="../trial_00/navigation_trace.json">原始导航轨迹</a> · <a href="../trial_00/pick/grasp_0794_trace.json">严格 Pick 轨迹</a></p><p>人工最终接受尚未填写；不会自动标为人工批准或导出最终数据集。</p></section></html>'''
    (folder/'review.html').write_text(page);return manifest
