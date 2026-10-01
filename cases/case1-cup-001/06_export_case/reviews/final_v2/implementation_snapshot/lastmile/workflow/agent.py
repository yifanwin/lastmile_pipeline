"""只读、多模态的场景建议；凭据只存在于内存，不得写出或充当人工审批。"""
import base64,json,os,urllib.request,urllib.error
from pathlib import Path
from .core import WORKSPACE,case_dir,read,write,digest,load_config,STAGES,now

def credentials():
 c=load_config('agents/scene_reviewer.yaml');path=WORKSPACE.parent/'.env';values={}
 from dotenv import dotenv_values
 if path.exists():values=dict(dotenv_values(path))
 get=lambda key:os.environ.get(key) or values.get(key)
 key,base,model=[get(c[k]) for k in ['key_env','base_url_env','model_env']]
 if not all([key,base,model]):raise ValueError('缺少 LLM_API_KEY / LLM_BASE_URL / LLM_MODEL；不输出配置值')
 if not base.startswith(('https://','http://')):raise ValueError('API endpoint scheme invalid')
 return c,key,base,model

def review_scene(case_id):
 folder=case_dir(case_id)/STAGES[0];facts=read(folder/'scene_facts.json');config,key,base,model=credentials()
 prompt='''你是场景审阅辅助 agent，不是人工审批人或物理验收器。只分析所附已恢复场景和几何事实，不能假设抓取或导航已经成功。输出一个 JSON 对象：target_assessment、tabletop_assessment、side_recommendations（north/south/east/west 各给理由）、detour_risks、recommendation（review/replace）、human_questions。所有可接近性和绕行判断是未验证建议；不得生成 human_approved、physics_verified 或人工审批。不得输出凭据、URL、代码、shell 命令。未知处明确写未知。'''
 content=[{'type':'text','text':prompt+'\n场景事实：'+json.dumps(facts,ensure_ascii=False)}]
 for name in ['overhead.png','initial_robot_view.png','target_closeup.png']:
  content.append({'type':'image_url','image_url':{'url':'data:image/png;base64,'+base64.b64encode((folder/'views'/name).read_bytes()).decode()}})
 body={'model':model,'messages':[{'role':'user','content':content}],'max_tokens':config['max_output_tokens']}
 url=base.rstrip('/')
 if not url.endswith('/chat/completions'):url+='/chat/completions'
 request=urllib.request.Request(url,data=json.dumps(body).encode(),headers={'Authorization':'Bearer '+key,'Content-Type':'application/json'},method='POST')
 try:
  with urllib.request.urlopen(request,timeout=config['timeout_s']) as response:payload=json.load(response)
  message=payload['choices'][0]['message']['content']
  if not isinstance(message,str) or not message.strip():raise ValueError('API output is empty or not text')
  # 防止供应端意外回显凭据；绝不记录原始 HTTP 响应或异常字符串。
  for sensitive in [key,base]:message=message.replace(sensitive,'[REDACTED]')
  clean=message.strip()
  if clean.startswith('```'):clean=clean.split('\n',1)[1].rsplit('```',1)[0].strip()
  try:advice=json.loads(clean)
  except json.JSONDecodeError:advice={'unstructured_advice':message}
  forbidden={'human_approved','physics_verified','decision','actor'}
  if isinstance(advice,dict):advice={k:v for k,v in advice.items() if k not in forbidden}
  result={'status':'advisory_only','actor':'agent','may_approve':False,'review_packet_sha256':digest(folder/'review_packet.json'),'created_at_utc':now(),'advice':advice}
 except Exception as exc:
  result={'status':'api_error','actor':'agent','may_approve':False,'error_type':type(exc).__name__,'http_status':exc.code if isinstance(exc,urllib.error.HTTPError) else None,'created_at_utc':now(),'note':'凭据、URL、原始响应和异常正文均不写入日志；人工仍可直接审阅'}
 write(folder/'agent_review.json',result);return result
