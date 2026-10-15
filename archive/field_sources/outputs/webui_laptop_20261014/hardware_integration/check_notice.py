"""DOM lifecycle regression; synthetic snapshots only, no task submission."""
import sys
sys.argv=['browser_check.py','--name','browser-readonly-before']
import browser_check as c
# Use the actual renderer with controlled snapshots, restoring live data last.
c.ws=c.websocket.create_connection(c.page['webSocketDebuggerUrl'],origin='http://localhost:9223',http_proxy_host=None,timeout=10)
c.call('Runtime.enable');c.call('Page.reload',{'ignoreCache':True});c.time.sleep(1)
r=c.evaluate('''(() => {
 const old=snapshot, oldId=taskNoticeId;
 const goal={mode:1,ordered_colors:[1],timeout_ms:180000};
 const steps=[];
 for (const status of ["SUBMITTING","ACCEPTED","EXECUTING","SUCCEEDED","ABORTED","CANCELED","UNKNOWN","REJECTED"]) {
  snapshot={...old,external_task:null,task:{task_id:"dom-fixture",goal,status,detail:"fixture-"+status,completed_count:0,total_count:1,completed_slots:[],elapsed_ms:0,result:null}};
  taskNoticeId="dom-fixture";renderTask();
  steps.push({status,notice:$("notice").textContent,hidden:$("notice").hidden});
 }
 snapshot=old;taskNoticeId=oldId;taskNoticeSignature="";$("notice").hidden=true;render();
 return steps;
})()''')
assert all(x['status'] in x['notice'] and not x['hidden'] for x in r),r
(c.OUT/'notice-regression.json').write_text(c.json.dumps(r,ensure_ascii=False,indent=2))
print(c.json.dumps(r,ensure_ascii=False));c.ws.close()
