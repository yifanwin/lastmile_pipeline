"""白名单组装的自包含证据包；草稿不具备人工接受/数据集准入标签。"""
import hashlib,json,os,shutil,zipfile,tempfile
from pathlib import Path
from . import core as c
from .construction import checked_file
from .final_review import require_review_packet,require_final_acceptance

def package_path(relative,sha,model_sha):
    p=Path(relative)
    if p.is_absolute() or '..' in p.parts or any(x.startswith('.') for x in p.parts):raise ValueError('非法/隐藏证据文件路径')
    if p.name=='compiled_model.mjb' and sha==model_sha:return 'assets/compiled_model.mjb'
    return str(Path('evidence')/p)

def verify_export_bundle(folder):
    folder=Path(folder);packet=c.read(folder/'package_manifest.json')
    for name,sha in packet['files'].items():checked_file(folder,name,sha)
    spec=c.read(folder/'case.json')
    if spec['dataset_admitted'] and not spec['human_final_accepted']:raise ValueError('包标签不一致：未经人工接受不能准入')
    if spec['status']=='accepted_export' and c.read(folder/'human_review.json')['decision']!='accepted':raise ValueError('包缺少人工接受')
    return {'all_file_hashes_valid':True,'files_verified':len(packet['files']),'status':spec['status'],'dataset_admitted':spec['dataset_admitted'],'human_final_accepted':spec['human_final_accepted']}

def assemble_case(case_id,review_id='final_v1',export_id='final_v1',draft=False):
    c.case_dir(export_id)
    if draft:review,packet,sha=require_review_packet(case_id,review_id);decision=None
    else:review,packet,decision=require_final_acceptance(case_id,review_id);sha=c.digest(review/'review_packet.json')
    stage=c.case_dir(case_id)/c.STAGES[5];parent=stage/('staging' if draft else 'exports');parent.mkdir(exist_ok=True);target=parent/export_id
    if target.exists() or (parent/f'{export_id}.zip').exists():raise ValueError('导出不可覆盖，请选择新 export-id')
    temporary=Path(tempfile.mkdtemp(dir=parent,prefix='.building-'));f=c.case_dir(case_id)/c.STAGES[0];model_sha=c.read(f/'review_packet.json')['compiled_model_sha256'];index={};copied={}
    try:
        for relative,file_sha in packet['source_files'].items():
            source=checked_file(c.WORKSPACE,relative,file_sha);dest_name=package_path(relative,file_sha,model_sha);dest=temporary/dest_name
            if dest_name in copied:
                if copied[dest_name]!=file_sha:raise ValueError('导出路径碰撞')
            else:
                dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(source,dest)
                if c.digest(dest)!=file_sha:raise ValueError('复制时证据被改变')
                copied[dest_name]=file_sha
            index[relative]={'packaged_path':dest_name,'sha256':file_sha}
        for name,file_sha in packet['review_files'].items():
            src=checked_file(review,name,file_sha);dest=temporary/'review'/name;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(src,dest)
        shutil.copyfile(review/'review_packet.json',temporary/'review/review_packet.json')
        source_draft=c.read(review/'case_draft.json');config=c.read(c.case_dir(case_id)/c.STAGES[3]/'runs'/packet['construction_run_id']/'case_config.json');root=c.case_dir(case_id)/c.STAGES[4]/'runs'/packet['closed_loop_run_id'];summary=c.read(root/'summary.json');asset=c.read(c.case_dir(case_id)/'inputs/asset.json')
        # grasp 与 MJCF 属于显式资产来源白名单，不遍历外部资产目录。
        for key,hash_key,dest in [('grasp_path','grasp_sha256','assets/grasp_source.npz'),('asset_xml','asset_xml_sha256','assets/asset_source.xml')]:
            if c.digest(asset[key])!=asset[hash_key]:raise ValueError('资产来源改变')
            destination=temporary/dest;destination.parent.mkdir(exist_ok=True);shutil.copyfile(asset[key],destination)
            if c.digest(destination)!=asset[hash_key]:raise ValueError('资产复制哈希错配')
        def ref(path):return index[str(Path(path).relative_to(c.WORKSPACE))]['packaged_path']
        trajectories=[]
        for row in summary['trials']:
            t=root/f'trial_{row["trial"]:02d}';trajectories.append({'trial':row['trial'],'seed':row['pick_backend_result']['seed'],'status':row['status'],'navigation_trace':ref(t/'navigation_trace.json'),'torso_adjust_trace':ref(t/'pick/torso_adjust_trace.json'),'pick_trace':ref(t/'pick/grasp_0794_trace.json'),'actual_arrival_snapshot':ref(t/'actual_arrival_snapshot.npz'),'saved_states':ref(t/'video_states.npz')})
        spec={**source_draft,'status':'draft_pending_human_acceptance' if draft else 'accepted_export','dataset_admitted':not draft,'human_final_accepted':not draft,'review_id':review_id,'review_packet_sha256':sha,'export_id':export_id,'scene':{'compiled_model':'assets/compiled_model.mjb','source_snapshot':ref(f/'initial_snapshot.npz'),'derived_initial_snapshot':ref(c.case_dir(case_id)/c.STAGES[3]/'runs'/packet['construction_run_id']/'case_initial_snapshot.npz')},'action_protocol':ref(c.WORKSPACE/'configs/protocols/protocol_v1.json'),'trajectories':trajectories,'video':ref(root/'delivery/case1_r_closed_loop.mp4'),'station_map':ref(c.case_dir(case_id)/c.STAGES[2]/'runs/coarse_v1/station_map.html'),'start_failure_certificate':ref(c.case_dir(case_id)/c.STAGES[3]/'runs'/packet['construction_run_id']/'reach_certificate.json'),'external_asset_meshes_bundled':False,'offline_replay_requires_external_asset_directory':False,'regeneration_requires_original_readonly_asset_directory':True,'generated_at_utc':c.now()}
        c.write(temporary/'case.json',spec);c.write(temporary/'evidence_index.json',index)
        if decision:c.write(temporary/'human_review.json',decision)
        else:c.write(temporary/'human_review.json',{'case_id':case_id,'review_id':review_id,'review_packet_sha256':sha,'decision':'pending','actor':'human','reviewer':''})
        from importlib.metadata import version,PackageNotFoundError
        environment={}
        for name in ['mujoco','numpy','torch','Pillow','scipy','curobo']:
            try:environment[name]=version(name)
            except PackageNotFoundError:environment[name]='local_source_not_package_metadata'
        c.write(temporary/'environment.json',environment)
        (temporary/'replay_saved_states.py').write_text(REPLAY_SCRIPT)
        (temporary/'README.md').write_text(f'''# {case_id} · {'草稿，未人工接受' if draft else '人工接受后正式导出'}

分类：Case 1-R，同侧平移可解决，不声称必须换边。完整链路 3/3 严格通过，按用户明确覆盖采用三次门槛。资产名义 5/5、小扰动 20/20 为已复核历史证据，不冒充新试验。

入口：[case.json](case.json)、[视频]({spec['video']})、[原始站位图]({spec['station_map']})、[起点范围证书]({spec['start_failure_certificate']})。完整导航/接触/支撑/保持记录和每次保存状态见 trajectories。

验证：`python replay_saved_states.py --verify-only`（标准库即可核验所有 SHA-256）。回放终态检查：`MUJOCO_GL=disable python replay_saved_states.py --trial 0`，需要 environment.json 中对应版本的 MuJoCo 和 NumPy，无需 NAS 资产。可 `--output frame.png` 渲染终态，需 EGL/OSMesa 可用。

compiled_model.mjb 已包含场景模型，原始和派生快照均保留。重新运行 cuRobo 生成轨迹仍需要原工程只读资产、GPU 与原依赖；此包不伪称包含所有外部资产。实现快照是导出时版本，当时执行文件哈希保留于 frozen_inputs.json，两者不混淆。

原始 JSON 路径字段保持原样以保存哈希；evidence_index.json 映射原工作区路径至本包文件。只复制白名单证据，不复制 .env、API 凭据、Git 元数据或其他场景。物理标签不可由人工覆盖。
''')
        hashes={str(p.relative_to(temporary)):c.digest(p) for p in temporary.rglob('*') if p.is_file()}
        c.write(temporary/'package_manifest.json',{'schema_version':1,'case_id':case_id,'export_id':export_id,'review_packet_sha256':sha,'status':spec['status'],'files':hashes})
        validation=verify_export_bundle(temporary)
        # 提交前再检查不可变来源和人工绑定，避免长复制期间证据变化。
        if draft:require_review_packet(case_id,review_id)
        else:require_final_acceptance(case_id,review_id)
        os.replace(temporary,target)
        archive=parent/f'{export_id}.zip';partial=parent/f'.{export_id}.zip.partial'
        with zipfile.ZipFile(partial,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=1,allowZip64=True) as z:
            for path in sorted(target.rglob('*')):
                if path.is_file():z.write(path,str(Path(export_id)/path.relative_to(target)))
        os.replace(partial,archive)
        receipt={'case_id':case_id,'review_id':review_id,'export_id':export_id,'status':spec['status'],'dataset_admitted':not draft,'bundle_path':str(target.relative_to(c.WORKSPACE)),'zip_path':str(archive.relative_to(c.WORKSPACE)),'zip_sha256':c.digest(archive),'package_manifest_sha256':c.digest(target/'package_manifest.json'),'files_verified':validation['files_verified'],'bytes':sum(p.stat().st_size for p in target.rglob('*') if p.is_file()),'completed_at_utc':c.now()}
        c.write(parent/f'{export_id}-receipt.json',receipt)
        if draft:c.update(case_id,c.STAGES[5],draft_ready=True,draft_bundle_path=receipt['bundle_path'],draft_zip_path=receipt['zip_path'])
        else:c.update(case_id,c.STAGES[5],status='exported',human_final_accepted=True,exported=True,bundle_path=receipt['bundle_path'],zip_path=receipt['zip_path'],zip_sha256=receipt['zip_sha256'],package_manifest_sha256=receipt['package_manifest_sha256'])
        return receipt
    except Exception:
        # 保留失败组装诊断，不覆盖任何已完成导出。
        if temporary.exists():(temporary/'BUILD_FAILED.txt').write_text('组装未完成；不得作为正式导出。\n')
        raise

REPLAY_SCRIPT=r'''"""可独立运行：SHA 检查、保存状态终态检查或渲染，无需源资产目录。"""
import argparse,hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--verify-only',action='store_true');p.add_argument('--trial',type=int,default=0);p.add_argument('--output');a=p.parse_args()
manifest=json.loads((ROOT/'package_manifest.json').read_text())
for name,sha in manifest['files'].items():
 f=(ROOT/name).resolve()
 if not f.is_relative_to(ROOT):raise ValueError('unsafe manifest path')
 h=hashlib.sha256()
 with f.open('rb') as stream:
  for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
 if h.hexdigest()!=sha:raise ValueError('hash mismatch: '+name)
print('all package file hashes valid:',len(manifest['files']))
if not a.verify_only:
 import numpy as np,mujoco
 spec=json.loads((ROOT/'case.json').read_text());trial=next(t for t in spec['trajectories'] if t['trial']==a.trial)
 model=mujoco.MjModel.from_binary_path(str(ROOT/spec['scene']['compiled_model']));data=mujoco.MjData(model);z=np.load(ROOT/trial['saved_states']);data.qpos[:]=z['qpos'][-1];data.time=z['time_s'][-1];mujoco.mj_forward(model,data)
 if np.any(np.diff(z['time_s'])<=0):raise ValueError('state time reset')
 print('replayed saved terminal state:',{'trial':a.trial,'physics_time_s':float(data.time),'nq':model.nq,'state_frames':len(z['time_s'])})
 if a.output:
  from PIL import Image
  model.vis.global_.offwidth=960;model.vis.global_.offheight=720
  with mujoco.Renderer(model,height=720,width=960) as renderer:
   camera=mujoco.MjvCamera();camera.lookat[:]=[7.35,1.65,.65];camera.distance=3.8;camera.azimuth=145;camera.elevation=-55;renderer.update_scene(data,camera=camera);Image.fromarray(renderer.render()).save(a.output)
'''
