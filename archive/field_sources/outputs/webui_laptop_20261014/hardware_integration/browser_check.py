"""CDP helper; default is read-only. Explicit --execute clicks exactly once."""
import argparse,base64,json,time,urllib.request
from pathlib import Path
import websocket
OUT=Path(__file__).resolve().parent
opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
pages=json.load(opener.open('http://127.0.0.1:9223/json'))
page=next(p for p in pages if p['type']=='page' and p['url']=='http://localhost:8767/')
ws=websocket.create_connection(page['webSocketDebuggerUrl'],origin='http://localhost:9223',http_proxy_host=None,timeout=10)
seq=0;exceptions=[]
def call(method,params=None):
 global seq
 seq+=1;ws.send(json.dumps(dict(id=seq,method=method,params=params or {})))
 while True:
  r=json.loads(ws.recv())
  if r.get('method')=='Runtime.exceptionThrown':exceptions.append(r)
  if r.get('id')==seq:
   if 'error' in r:raise RuntimeError(r['error'])
   return r.get('result')
def evaluate(expression):
 r=call('Runtime.evaluate',dict(expression=expression,returnByValue=True,awaitPromise=True))
 if r.get('exceptionDetails'):raise RuntimeError(r['exceptionDetails'])
 return r['result'].get('value')
def state():
 return evaluate('({mode:snapshot?.mode,status:snapshot?.task?.status,task:snapshot?.task?.task_id,detail:snapshot?.task?.detail,notice:$("notice").textContent,noticeHidden:$("notice").hidden,submitDisabled:$("submit").disabled,connection:$("connection").textContent,joints:$("joints").textContent,robotStop:$("robot-stop").textContent,operations:snapshot?.operations})')
p=argparse.ArgumentParser();p.add_argument('--execute',action='store_true');p.add_argument('--watch',type=int,default=0);p.add_argument('--name',default='browser-state');a=p.parse_args()
call('Runtime.enable');call('Page.enable')
if a.execute:
 s=state();assert s['mode']=='hardware_trajectory' and not s['submitDisabled'] and not s.get('task'),s
 # No other click and no HTTP retry path.
 evaluate('$("timeout").value="180"; $("task-form").requestSubmit(); "submitted_once"')
trace=[]
for _ in range(max(1,a.watch)):
 trace.append(dict(unix=time.time(),state=state()))
 if a.watch:time.sleep(1)
(OUT/(a.name+'.json')).write_text(json.dumps(dict(trace=trace,exceptions=exceptions),ensure_ascii=False,indent=2))
r=call('Page.captureScreenshot',{'format':'png','captureBeyondViewport':True})
(OUT/(a.name+'.png')).write_bytes(base64.b64decode(r['data']))
print(json.dumps(trace[-1],ensure_ascii=False,indent=2));print('exceptions',len(exceptions))
ws.close()
