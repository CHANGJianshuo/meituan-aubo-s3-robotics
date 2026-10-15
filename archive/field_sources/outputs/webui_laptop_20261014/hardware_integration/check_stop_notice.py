"""Read-only browser regression. api/refresh are replaced in this tab only."""
import sys
sys.argv=['browser_check.py','--name','browser-completed-before-refresh']
import browser_check as c
c.ws=c.websocket.create_connection(c.page['webSocketDebuggerUrl'],origin='http://localhost:9223',http_proxy_host=None,timeout=10)
c.call('Runtime.enable');c.call('Page.reload',{'ignoreCache':True});c.time.sleep(1)
checks=c.evaluate('''(async () => {
 const saved={api,refresh,snapshot,taskNoticeId,taskNoticeSignature,busy,online,paused,lastReceived};
 const steps=[];
 try {
   api=async()=>({request_delivered:true}); refresh=async()=>{};
   snapshot={...snapshot,task:{...snapshot.task,status:"EXECUTING",detail:"fixture-running"}};
   online=true;paused=false;busy=false;lastReceived=performance.now();
   await command("/api/stop",{},"fixture-stop-pending");
   snapshot.task={...snapshot.task,status:"CANCELED",detail:"fixture-controller-stopped"};
   renderTask();
   steps.push({test:"stop terminal notice",notice:$("notice").textContent,pass:$("notice").textContent.includes("已取消")&&$("notice").textContent.includes("fixture-controller-stopped")});
 } finally {
   api=saved.api;refresh=saved.refresh;snapshot=saved.snapshot;taskNoticeId=snapshot.task.task_id;
   taskNoticeSignature="";busy=saved.busy;online=saved.online;paused=saved.paused;lastReceived=saved.lastReceived;
   awaitingSnapshot=false;render();
 }
 return steps;
})()''')
assert all(x['pass'] for x in checks),checks
(c.OUT/'stop-notice-regression.json').write_text(c.json.dumps(checks,ensure_ascii=False,indent=2));print(checks)
state=c.state();assert state['status']=='SUCCEEDED' and state['notice'].startswith('已完成'),state
(c.OUT/'browser-final.json').write_text(c.json.dumps(dict(state=state,exceptions=c.exceptions),ensure_ascii=False,indent=2))
r=c.call('Page.captureScreenshot',{'format':'png','captureBeyondViewport':True});(c.OUT/'browser-final.png').write_bytes(c.base64.b64decode(r['data']))
print(c.json.dumps(state,ensure_ascii=False,indent=2));c.ws.close()
