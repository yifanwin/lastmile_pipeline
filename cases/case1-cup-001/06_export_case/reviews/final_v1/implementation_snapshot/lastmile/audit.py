"""只读提取 benchmark、场景、资产和 grasp，生成可追溯审计记录。"""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

BENCHMARK = 'benchmarks/molmospaces-bench-v1/procthor-10k/RBY1PickAndPlaceDataGenConfig/RBY1PickAndPlaceDataGenConfig_20260209_json_benchmark/benchmark.json'


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)+'\n')


def extract(assets, workspace):
    assets, workspace = Path(assets).absolute(), Path(workspace).absolute()
    whitelist_path=workspace/'registry/whitelist.json'
    if whitelist_path.exists() and json.loads(whitelist_path.read_text()).get('assets'):
        raise ValueError('正式白名单已非空；拒绝用审计脚本覆盖物理准入记录')
    source = assets/BENCHMARK
    source_hash = sha256(source)
    episodes = json.loads(source.read_text())
    candidates = json.loads((workspace/'configs/candidates.json').read_text())
    rows, unique = [], {}
    for candidate in candidates:
        index = candidate['source_index']
        episode = episodes[index]
        scene = assets/f"scenes/{episode['scene_dataset']}-{episode['data_split']}/{episode['data_split']}_{episode['house_index']}.xml"
        metadata_path = scene.with_name(scene.stem+'_metadata.json')
        metadata = json.loads(metadata_path.read_text())['objects']
        target = episode['task']['pickup_obj_name']
        obj = metadata[target]
        uid = obj['asset_id']
        if (episode['house_index'], uid) != (candidate['expected_house'], candidate['expected_asset']):
            raise ValueError(f'候选索引错配：{index}')
        root = ET.parse(scene).getroot()
        target_body = next(b for b in root.iter('body') if b.get('name') == target)
        scene_meshes = {m.get('name'):m for m in root.iter('mesh')}
        used = [scene_meshes[g.get('mesh')] for g in target_body.iter('geom') if g.get('mesh')]
        mesh_path = scene.parent/used[0].get('file')
        # THOR Prefabs 资产目录以 uid 命名；不根据 hash_name 推断 uid。
        asset_dir = next(p for p in mesh_path.parents if p.name == uid)
        asset_xml = asset_dir/(uid+'.xml')
        parent = obj.get('parent')
        support = metadata.get(parent, {})
        row = dict(candidate, asset_id=uid, target_name=target,
                   house_index=episode['house_index'], source=episode['source'],
                   benchmark_sha256=source_hash, scene_xml=str(scene),
                   scene_sha256=sha256(scene), metadata_sha256=sha256(metadata_path),
                   support_parent=parent, support_category=support.get('category'),
                   support_asset_id=support.get('asset_id'),
                   support_status='metadata_parent_only_not_physical_contact_verified',
                   source_pose=episode['task']['pickup_obj_start_pose'],
                   physics_status='not_tested')
        rows.append(row)
        # 完整恢复相关字段；不继续使用 PnP task 的成功条件。
        frozen = {k:v for k,v in episode.items() if k != 'task'}
        frozen['task'] = {k:episode['task'][k] for k in ('robot_base_pose','pickup_obj_name','pickup_obj_start_pose')}
        frozen['task'].update(task_type='strict_pick', contents_simulated=False)
        frozen['provenance'] = row
        frozen['modifications'] = ['replace PnP task verdict with strict Pick; keep scene objects unchanged']
        write_json(workspace/f'registry/episodes/{index:04d}.json', frozen)
        if uid in unique:
            unique[uid]['source_indexes'].append(index)
            continue
        grasp_path = assets/f'grasps/droid/{uid}/{uid}_grasps_filtered.npz'
        with np.load(grasp_path, allow_pickle=False) as archive:
            transforms = archive['transforms'].astype(np.float64)
            keys = archive.files
        if transforms.ndim != 3 or transforms.shape[1:] != (4,4):
            raise ValueError(f'grasp shape invalid: {uid}')
        rotations = transforms[:,:3,:3]
        valid = (np.isfinite(transforms).all(axis=(1,2)) &
                 np.isclose(np.linalg.det(rotations), 1., atol=2e-3) &
                 np.isclose(rotations.transpose(0,2,1)@rotations, np.eye(3), atol=2e-3).all(axis=(1,2)) &
                 np.isclose(transforms[:,3,:], [0,0,0,1], atol=2e-3).all(axis=1))
        ar = ET.parse(asset_xml).getroot()
        geoms = [{k:g.get(k) for k in ('name','type','size','pos','quat','mesh','class')}
                 for g in ar.iter('geom')]
        record = dict(asset_id=uid, source_indexes=[index],
                      asset_xml=str(asset_xml), asset_xml_sha256=sha256(asset_xml),
                      grasp_path=str(grasp_path), grasp_sha256=sha256(grasp_path),
                      grasp_npz_keys=keys, grasp_count=len(transforms),
                      valid_transform_count=int(valid.sum()),
                      candidate_grasp_ids=np.flatnonzero(valid).tolist(),
                      grasp_id_definition='row index in transforms; no random subsampling, no implicit flips',
                      gripper_compatibility='droid grasps; RBY1 finger/contact compatibility not verified',
                      mesh_scale={m.get('name'):m.get('scale','1 1 1') for m in ar.iter('mesh')},
                      source_collision_geoms=[g.get('name') for g in target_body.iter('geom')
                                             if g.get('class') != '__VISUAL_MJT__'],
                      asset_geoms=geoms, handle_annotation_status='pending_human_and_contact_validation' if candidate['mode']=='handle' else 'not_applicable',
                      handle_allowed_grasp_ids=[],
                      admission_status='pending_physics', rejection_reason=None)
        unique[uid] = record
    write_json(workspace/'registry/candidates.json', rows)
    for uid, record in unique.items():
        path=workspace/f'registry/assets/{uid}.json'
        if path.exists():
            prior=json.loads(path.read_text())
            if prior.get('grasp_sha256')==record['grasp_sha256'] and prior.get('asset_xml_sha256')==record['asset_xml_sha256']:
                for key in ('handle_annotation_status','handle_allowed_grasp_ids','part_regions','cup_up_axis','annotation_review','admission_status','rejection_reason'):
                    if key in prior:record[key]=prior[key]
        write_json(path, record)
    write_json(workspace/'registry/whitelist.json', {
        'status':'no_physical_admissions', 'protocol_sha256':sha256(workspace/'configs/protocol_v1.json'),
        'assets':[], 'note':'静态合法 grasp 不等于物理稳定；未测试不能标为拒绝或不可达'})
    write_json(workspace/'registry/provenance.json', {
        'benchmark':str(source),'benchmark_sha256':source_hash,'episode_count':len(episodes),
        'plan_sha256':sha256(workspace.parent/'lastmile case 构建方案.md'),
        'candidate_count':len(rows),'unique_asset_count':len(unique)})
    return rows, list(unique.values())
