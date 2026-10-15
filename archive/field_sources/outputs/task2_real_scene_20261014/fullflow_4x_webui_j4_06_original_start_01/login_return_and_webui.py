#!/usr/bin/env python3
"""One login: await an explicit checked return request, then connect the task UI.

Only the return is agent-authorized. The original thirty-stage task is never
submitted here. Credentials remain in memory and travel only through stdin.
"""
from datetime import datetime, timezone
import getpass
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import urllib.request

ROOT=Path(__file__).resolve().parent
WORK=ROOT.parents[2]
RESET=ROOT/'return_to_start'
WEBUI=WORK/'outputs/webui_laptop_20261014'
sys.path.insert(0,str(WORK/'scripts/task2_real_scene'))
from field_session import run_stage, validate_request


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    os.umask(0o077)
    folder=RESET/'field_session'
    folder.mkdir(mode=0o700)
    state=dict(pid=os.getpid(),status='awaiting_login',motion_commands_sent=0,
               task_automatic_submission=False)
    def status(value,**fields):
        state.update(status=value,updated_utc=datetime.now(timezone.utc).isoformat(),**fields)
        pending=folder/'status.tmp';pending.write_text(json.dumps(state,indent=2)+'\n')
        pending.replace(folder/'status.json')
    status('awaiting_login')
    settings=json.loads((ROOT/'hardware_config.json').read_text())
    reset_deployment=json.loads((RESET/'deployment.json').read_text())
    assert reset_deployment['status']=='deployed_and_offline_verified'
    entry=json.loads((RESET/'commissioning/commissioning.json').read_text())['packages'][0]
    assert entry['scope']=='separate_unloaded_return'
    assert json.loads((ROOT/'request.json').read_text())['reset_authorization']=='你直接复位，不需要我操作'
    task_manifest=json.loads((Path(settings['package'])/'manifest.json').read_text())
    assert sha(Path(settings['package'])/'manifest.json')==settings['manifest_sha256']
    assert task_manifest['task_start_q']==entry['end_q']
    stop=threading.Event();credentials=None
    opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
    def snapshot():return json.load(opener.open('http://127.0.0.1:8767/api/snapshot',timeout=5))
    def stopped_at(q):
        s=snapshot();r=s['robot'];d=r['data']
        assert r['fresh'] and d['stop_confirmed'] and not d['motion_active']
        assert d['exec_id']==-1 and d['queue']==0 and max(map(abs,d['qd']))<math.radians(.02)
        assert max(abs(a-b) for a,b in zip(d['q'],q))<=math.radians(.05)
        assert (s.get('task') or {}).get('status') not in ('SUBMITTING','ACCEPTED','EXECUTING','CANCELING','UNKNOWN')
        assert not s['operations']['submit']
        return s
    try:
        stopped_at(entry['start_q'])
        print('任务二：先执行已核验的独立复位，再恢复原轨迹网页。',flush=True)
        print('登录后等待助手发送单次复位请求；完整任务仍由你在网页点击启动。Ctrl+C 停止复位。',flush=True)
        credentials=dict(username=input('SDK username [aubo]: ').strip() or 'aubo',
                         password=getpass.getpass('SDK password (not saved): '))
        signal.signal(signal.SIGINT,lambda *_:stop.set())
        signal.signal(signal.SIGTERM,lambda *_:stop.set())
        status('awaiting_return_request')
        request_path=folder/'request.json'
        while not request_path.exists() and not stop.wait(.2):
            if (folder/'stop.request').exists():stop.set()
        if stop.is_set():status('stopped_before_motion');return
        request=json.loads(request_path.read_text())
        validate_request(request,entry,RESET)
        stopped_at(entry['start_q'])
        request_path.rename(folder/'return.request.json')
        status('starting_return')
        log=Path(run_stage(RESET,folder,reset_deployment,entry,credentials,stop,status))
        events=[json.loads(line) for line in log.read_text().splitlines()]
        endings=[e for e in events if e['event']=='stage_complete']
        responses=[e for e in events if e['event']=='move_response']
        assert [e['stage'] for e in endings]==[1,2]
        assert [e['stage'] for e in responses]==[1,2] and all(e['result']==0 for e in responses)
        assert any(e['event']=='completed' and e['plan_id']==entry['plan_id'] for e in events)
        assert any(e['event']=='player_returned' for e in events)
        assert not stop.is_set()
        status('verifying_return_endpoint',motion_commands_sent=2)
        samples=[];until=time.monotonic()+12
        while time.monotonic()<until and len(samples)<4:
            s=stopped_at(entry['end_q'])
            if not samples or s['robot']['source_stamp_ns']!=samples[-1]['robot']['source_stamp_ns']:
                samples.append(s)
            time.sleep(.5)
        assert len(samples)==4 and samples[-1]['robot']['source_stamp_ns']>samples[0]['robot']['source_stamp_ns']
        proof=dict(passed=True,return_only=True,completed_stages=2,task_motion_commands_sent=0,
            receipt=str(log),receipt_sha256=sha(log),independent_final_telemetry=samples)
        (RESET/'physical_return_validation.json').write_text(json.dumps(proof,indent=2)+'\n')
        status('return_completed_connecting_original_task_ui')
        subprocess.run([sys.executable,str(WEBUI/'start.py'),'--stop'],check=True)
        destination=WEBUI/'hardware_integration/hardware_config.json'
        (ROOT/'previous_ui_config.json').write_bytes(destination.read_bytes())
        pending=destination.with_name('task2-original-start.pending.json')
        with pending.open('x') as f:json.dump(settings,f,ensure_ascii=False,indent=2)
        pending.replace(destination)
        subprocess.run([sys.executable,str(WEBUI/'start.py'),'--auth-stdin','--open-browser'],
            input=(json.dumps(credentials)+'\n').encode(),check=True)
        status('return_completed_original_task_ui_connected',task_motion_commands_sent=0)
        print('已复位，原始 30 段轨迹已连接：http://localhost:8767/；现场核对后由你点击启动。',flush=True)
    except BaseException as error:
        status('failed_no_retry',error=type(error).__name__+': '+str(error))
        raise
    finally:
        if credentials is not None:credentials.clear()


if __name__=='__main__':
    try:main()
    except BaseException as e:
        print(type(e).__name__+': '+str(e),file=sys.stderr,flush=True)
        raise SystemExit(1)
