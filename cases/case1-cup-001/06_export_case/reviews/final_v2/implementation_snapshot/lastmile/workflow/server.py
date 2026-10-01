from http.server import HTTPServer,SimpleHTTPRequestHandler
from functools import partial
from .core import *
def serve(case_id,port):
 folder=case_dir(case_id)/STAGES[0];review_packet_valid(case_id)
 class Handler(SimpleHTTPRequestHandler):
  def translate_path(self,path):
   candidate=Path(super().translate_path(path)).resolve()
   if not candidate.is_relative_to(folder.resolve()):return str(folder/'__forbidden__')
   return str(candidate)
  def do_POST(self):
   try:
    if self.path!='/api/review':raise ValueError('未知提交路径')
    if self.headers.get('Origin') not in [None,f'http://127.0.0.1:{port}',f'http://localhost:{port}']:raise ValueError('拒绝跨站提交')
    if self.headers.get_content_type()!='application/json':raise ValueError('需要 JSON')
    length=int(self.headers.get('Content-Length','0'))
    if not 0<length<=16384:raise ValueError('请求大小不合法')
    data=json.loads(self.rfile.read(length))
    if data.get('case_id')!=case_id:raise ValueError('case_id 不匹配')
    result=confirm_review(case_id,data)
    from .restore import render_card
    render_card(case_id);status=200
   except Exception as e:result={'error':str(e)};status=400
   payload=json.dumps(result,ensure_ascii=False).encode();self.send_response(status);self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(payload)));self.end_headers();self.wfile.write(payload)
 print(f'人工审阅：http://127.0.0.1:{port}/review.html',flush=True)
 HTTPServer(('127.0.0.1',port),partial(Handler,directory=str(folder))).serve_forever()

def serve_map(case_id,run_id,port):
 case_dir(run_id);folder=case_dir(case_id)/STAGES[2]/'runs'/run_id
 if not (folder/'station_map.html').exists():raise ValueError('站位图尚未生成')
 class ReadOnlyHandler(SimpleHTTPRequestHandler):
  def translate_path(self,path):
   candidate=Path(super().translate_path(path)).resolve()
   return str(candidate) if candidate.is_relative_to(folder.resolve()) else str(folder/'__forbidden__')
  def do_POST(self):self.send_error(405,'Read-only map')
 server=HTTPServer(('127.0.0.1',port),partial(ReadOnlyHandler,directory=str(folder)))
 print(f'站位图：http://127.0.0.1:{port}/station_map.html',flush=True);server.serve_forever()

def final_review_handler(case_id,review_id,port):
 from urllib.parse import urlsplit,unquote
 from .final_review import require_review_packet,confirm_final_review
 folder,packet,_=require_review_packet(case_id,review_id)
 class FinalHandler(SimpleHTTPRequestHandler):
  def translate_path(self,path):
   name=unquote(urlsplit(path).path)
   if name.startswith('/evidence/'):
    relative=name[len('/evidence/'):]
    if relative not in packet['source_files']:return str(folder/'__forbidden__')
    candidate=(WORKSPACE/relative).resolve()
    return str(candidate) if candidate.is_relative_to(WORKSPACE.resolve()) else str(folder/'__forbidden__')
   candidate=Path(super().translate_path(path)).resolve()
   return str(candidate) if candidate.is_relative_to(folder.resolve()) else str(folder/'__forbidden__')
  def do_POST(self):
   try:
    if self.path!='/api/final-review':raise ValueError('未知提交路径')
    if self.headers.get('Origin') not in [None,f'http://127.0.0.1:{port}',f'http://localhost:{port}']:raise ValueError('拒绝跨站提交')
    if self.headers.get_content_type()!='application/json':raise ValueError('需要 JSON')
    size=int(self.headers.get('Content-Length','0'))
    if not 0<size<=16384:raise ValueError('请求大小不合法')
    data=json.loads(self.rfile.read(size))
    if data.get('review_id')!=review_id:raise ValueError('review_id 错配')
    result=confirm_final_review(case_id,review_id,data);status=200
   except Exception as exc:result={'error':str(exc)};status=400
   raw=json.dumps(result,ensure_ascii=False).encode();self.send_response(status);self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(raw)));self.end_headers();self.wfile.write(raw)
 return partial(FinalHandler,directory=str(folder))

def serve_final_review(case_id,review_id,port):
 server=HTTPServer(('127.0.0.1',port),final_review_handler(case_id,review_id,port))
 print(f'最终人工审阅：http://127.0.0.1:{port}/review.html',flush=True);server.serve_forever()
