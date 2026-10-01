from pathlib import Path
import hashlib,json,os,re,tempfile
from datetime import datetime,timezone
import yaml
WORKSPACE=Path(__file__).resolve().parents[3]
LEGACY=WORKSPACE/'archive/calibration_v1'
STAGES=['01_restore_review','02_stable_grasp','03_station_map','04_construct_start','05_closed_loop','06_export_case']

def read(path):return json.loads(Path(path).read_text())
def digest(path):
 h=hashlib.sha256()
 with Path(path).open('rb') as f:
  for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
 return h.hexdigest()
def write(path,obj):
 path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
 fd,tmp=tempfile.mkstemp(dir=path.parent,prefix='.writing-',suffix='.json')
 try:
  with os.fdopen(fd,'w') as f:json.dump(obj,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')
  os.replace(tmp,path)
 finally:
  if os.path.exists(tmp):os.unlink(tmp)
def now():return datetime.now(timezone.utc).isoformat()
def case_dir(case_id):
 if not re.fullmatch(r'[a-z0-9][a-z0-9_-]{0,63}',case_id):raise ValueError('case_id 必须为安全的小写字母/数字/下划线/连字符')
 return WORKSPACE/'cases'/case_id

def load_config(name):return yaml.safe_load((WORKSPACE/'configs'/name).read_text())
def remap_legacy(path):
 p=Path(path)
 if p.exists():return p
 for prefix in [WORKSPACE,Path('/home/wenyifan/wenyifan/MoMaTrajGen/lastmile_pipeline')]:
  try:relative=p.relative_to(prefix)
  except ValueError:continue
  if relative.parts and relative.parts[0] in ['runs','reports','registry','configs','scripts']:
   candidate=LEGACY/relative
   if candidate.exists():return candidate
 raise FileNotFoundError('历史证据路径不存在：'+str(path))

def manifest(case_id):
 p=case_dir(case_id)/'manifest.json'
 return read(p) if p.exists() else {'schema_version':1,'case_id':case_id,'case_type':'case1_distance','created_at_utc':now(),'stages':{s:{'status':'not_run'} for s in STAGES}}
def update(case_id,stage,**fields):
 m=manifest(case_id);m['stages'][stage].update(fields);m['updated_at_utc']=now();write(case_dir(case_id)/'manifest.json',m);return m

def review_packet_valid(case_id):
 folder=case_dir(case_id)/STAGES[0];packet=read(folder/'review_packet.json')
 for name,sha in packet['files'].items():
  p=folder/name
  if not p.exists() or digest(p)!=sha:raise ValueError('场景审阅证据已改变/缺失：'+name)
 for name,key in [('compiled_model.mjb','compiled_model_sha256'),('initial_snapshot.npz','initial_snapshot_sha256'),('../inputs/source_episode.json','source_episode_sha256')]:
  if digest(folder/name)!=packet[key]:raise ValueError('恢复模型/快照/源记录已改变')
 return digest(folder/'review_packet.json')

def confirm_review(case_id,decision):
 sha=review_packet_valid(case_id)
 if decision.get('review_packet_sha256')!=sha:raise ValueError('人工确认对应旧版审阅卡，拒绝推进')
 action=decision.get('decision')
 if action not in ['approved','rejected']:raise ValueError('只能由人工提交 approved/rejected')
 if not str(decision.get('reviewer','')).strip():raise ValueError('需要填写人工审阅者')
 if action=='approved':
  if decision.get('target_correct') is not True or decision.get('is_tabletop') is not True:raise ValueError('必须确认目标和桌面正确')
  sides=decision.get('suitable_sides',[])
  if not sides or any(s not in ['north','south','east','west'] for s in sides):raise ValueError('需要至少一个合理操作桌边')
  if decision.get('retain_case_type')!='case1_distance':raise ValueError('目前只接受 Case 1 距离型')
 folder=case_dir(case_id)/STAGES[0];previous=folder/'human_review.json'
 if previous.exists():write(folder/'review_history'/f'{now().replace(":","-")}.json',read(previous))
 saved={**decision,'actor':'human','submitted_at_utc':now()};write(previous,saved)
 m=update(case_id,STAGES[0],status=action,human_approved=action=='approved',review_packet_sha256=sha)
 if m['stages'][STAGES[1]].get('physics_verified'):
  summary_path=case_dir(case_id)/STAGES[1]/'summary.json'
  if summary_path.exists():
   summary=read(summary_path);summary.update(human_approved=action=='approved',status='verified' if action=='approved' else 'provisional_verified');write(summary_path,summary)
  update(case_id,STAGES[1],status='verified' if action=='approved' else 'provisional_verified')
 return saved

def require_review(case_id):
 folder=case_dir(case_id)/STAGES[0];sha=review_packet_valid(case_id);p=folder/'human_review.json'
 if not p.exists():raise ValueError('步骤 1 尚未人工确认')
 h=read(p)
 if h.get('actor')!='human' or h.get('decision')!='approved' or h.get('review_packet_sha256')!=sha:raise ValueError('步骤 1 人工确认缺失、拒绝或过期')
 return h

def require_next_stage(case_id):
 require_review(case_id);m=manifest(case_id)
 if not m['stages'][STAGES[1]].get('physics_verified'):raise ValueError('步骤 2 的 A 点稳定抓法尚未物理通过')
 folder=case_dir(case_id)/STAGES[1];packet=read(folder/'physics_evidence.json')
 for name,sha in packet['files'].items():
  if digest(folder/name)!=sha:raise ValueError('步骤 2 物理证据已改变')
 history=read(folder/'historical_revalidation.json')
 for evidence in history['traces']:
  if digest(WORKSPACE/evidence['trace_path'])!=evidence['trace_sha256']:raise ValueError('历史物理轨迹已改变')
 summary=read(folder/'summary.json')
 if summary.get('physics_verified') is not True or summary['fresh_A_witness']['status']!='verified_success':raise ValueError('步骤 2 未通过')
 if digest(WORKSPACE/'configs/protocols/protocol_v1.json')!=summary['protocol_sha256']:raise ValueError('验收协议已变化')
 return True

def engine_context(case_id,episode,asset):
 context=case_dir(case_id)/'inputs/engine_context'
 write(context/f'registry/episodes/{episode["provenance"]["source_index"]:04d}.json',episode)
 write(context/f'registry/assets/{asset["asset_id"]}.json',asset)
 p=context/'configs/protocol_v1.json';p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes((WORKSPACE/'configs/protocols/protocol_v1.json').read_bytes())
 return context
