"""Click the prepared Web UI once, after its independent guards enable it."""
from datetime import datetime,timezone
import json
from pathlib import Path
import time
import urllib.request
import websocket

ROOT=Path(__file__).resolve().parent
PLAN='282b540c0f054c0ea39eb2962ce095de'
opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
def read(url):return json.load(opener.open(url,timeout=5))
state=read('http://127.0.0.1:8767/api/snapshot')
assert state['config']['plan_id']==PLAN
assert state['task'] is None, 'A task already exists; do not submit another'
pages=read('http://127.0.0.1:9223/json')
page=next(p for p in pages if p['type']=='page' and p['url'].startswith('http://localhost:8767/'))
socket=websocket.create_connection(page['webSocketDebuggerUrl'],origin='http://localhost:9223',http_proxy_host=None,timeout=10)
seq=0
def call(method,params=None):
    global seq
    seq+=1;socket.send(json.dumps(dict(id=seq,method=method,params=params or {})))
    while True:
        result=json.loads(socket.recv())
        if result.get('id')==seq:
            assert 'error' not in result,result
            return result.get('result')
call('Page.enable');call('Runtime.enable');call('Page.reload',{'ignoreCache':True})
time.sleep(1)
deadline=time.monotonic()+45
while True:
    value=call('Runtime.evaluate',dict(expression='''({
 plan:snapshot?.config?.plan_id,task:snapshot?.task,enabled:!$("submit").disabled,
 mode:draftGoal().mode,colors:draftGoal().ordered_colors,timeout:Number($("timeout").value),
 fresh:transportFresh(),camera:snapshot?.camera?.color?.fresh,robot:snapshot?.robot?.fresh
})''',returnByValue=True))
    if value.get('exceptionDetails'):
        raise RuntimeError('Browser state unavailable; no submission')
    before=value['result']['value']
    assert before['task'] is None, 'Another task appeared; do not submit'
    if before['enabled'] and before['fresh'] and before['camera'] and before['robot']:break
    if time.monotonic()>deadline:raise RuntimeError('UI guards did not become ready; no submission')
    time.sleep(.1)
assert before['plan']==PLAN and before['task'] is None and before['enabled']
assert before['mode']==2 and before['colors']==[1,3,2]
assert before['fresh'] and before['camera'] and before['robot'] and before['timeout']>=264
intent=dict(utc=datetime.now(timezone.utc).isoformat(),plan_id=PLAN,target_id=page['id'],
    method='One click on the actual task-two Web UI submit button',before=before,
    automatic_retry=False)
with (ROOT/'ui_submit_intent.json').open('x') as file:json.dump(intent,file,indent=2);file.write('\n')
result=call('Runtime.evaluate',dict(expression='''(()=>{
 if(snapshot.config.plan_id!=="282b540c0f054c0ea39eb2962ce095de" || snapshot.task || $("submit").disabled) throw new Error("UI changed before submission");
 $("submit").click();return true;
})()''',returnByValue=True))
assert not result.get('exceptionDetails'),result
socket.close()
time.sleep(1)
after=read('http://127.0.0.1:8767/api/snapshot')
with (ROOT/'ui_after_submit.json').open('x') as file:json.dump(after,file,ensure_ascii=False,indent=2);file.write('\n')
print(json.dumps(dict(task=after.get('task'),operations=after.get('operations')),ensure_ascii=False,indent=2))
