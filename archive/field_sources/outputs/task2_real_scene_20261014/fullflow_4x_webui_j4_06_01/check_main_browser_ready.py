"""Inspect the authenticated main UI through CDP; never submit a task."""
from pathlib import Path
import base64
import json
import time
import urllib.request
import websocket

root=Path(__file__).resolve().parent
opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
pages=json.load(opener.open('http://127.0.0.1:9223/json'))
page=next(p for p in pages if p.get('url','').rstrip('/') in ('http://localhost:8767','http://127.0.0.1:8767'))
target=page['id']
socket=websocket.create_connection(page['webSocketDebuggerUrl'],origin='http://localhost:9223',http_proxy_host=None,timeout=10)
seq=0;errors=[]
def call(method,params=None):
    global seq
    seq+=1;socket.send(json.dumps(dict(id=seq,method=method,params=params or {})))
    while True:
        result=json.loads(socket.recv())
        if result.get('method')=='Runtime.exceptionThrown':errors.append(result)
        if result.get('id')==seq:
            assert 'error' not in result,result
            return result.get('result')
call('Runtime.enable');call('Page.enable');call('Page.reload',{'ignoreCache':True});time.sleep(2)
result=call('Runtime.evaluate',dict(expression='''({
 guards:snapshot.config.tracking_guard_deg,mode:snapshot?.mode,motionSeconds:snapshot.config.nominal_motion_seconds,planId:snapshot.config.plan_id,goal:snapshot?.config?.supported_goal,title:$("environment-note").textContent,
 selectedMode:document.querySelector("input[name=mode]:checked").value,
 colors:[0,1,2].map(i=>$("color-"+i).value),disabled:[0,1,2].map(i=>$("color-"+i).disabled),
 progress:$("progress-count").textContent,submit:$("submit").textContent,
 submitDisabled:$("submit").disabled,cameraFresh:snapshot.camera.color.fresh,cameraVisible:!$("camera-image").hidden,cameraWidth:$("camera-image").naturalWidth,overflow:document.documentElement.scrollWidth>innerWidth
})''',returnByValue=True))
assert not result.get('exceptionDetails'),result
value=result['result']['value']
passed=(abs(value['motionSeconds']-114.0094890693904)<1e-6 and value['planId']=='139792522d8045f18e727579ab348664' and value['goal']=={'mode':2,'ordered_colors':[1,3,2]} and value['colors']==['1','3','2']
    and value['guards']==[.5,.5,.5,.6,.5,.5] and 'J4 0.6°' in value['title'] and value['selectedMode']=='2' and value['progress']=='0 / 30'
    and not value['submitDisabled'] and value['cameraFresh'] and value['cameraVisible'] and value['cameraWidth']>0 and all(value['disabled']) and not value['overflow'] and not errors)
screen=call('Page.captureScreenshot',{'format':'png','captureBeyondViewport':True})
(root/'webui_manual_submit_ready.png').write_bytes(base64.b64decode(screen['data']))
(root/'webui_manual_submit_browser_check.json').write_text(json.dumps(dict(passed=passed,read_only=True,
    motion_commands_sent=0,target_id=target,state=value,exceptions=errors),ensure_ascii=False,indent=2)+'\n')
socket.close();print(json.dumps(value,ensure_ascii=False,indent=2))
raise SystemExit(0 if passed else 1)
