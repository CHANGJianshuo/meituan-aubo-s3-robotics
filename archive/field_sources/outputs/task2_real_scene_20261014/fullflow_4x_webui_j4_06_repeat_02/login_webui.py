#!/usr/bin/env python3
"""Connect the prepared task-two UI; scene review still gates all motion."""
from datetime import datetime, timezone
import getpass
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import urllib.request

ROOT=Path(__file__).resolve().parent
WORKSPACE=ROOT.parents[2]
WEBUI=WORKSPACE/'outputs/webui_laptop_20261014'


def main():
    assert json.loads((ROOT/'reset_scene_check.json').read_text())['passed']
    assert json.loads((ROOT/'previous_success_check.json').read_text())['passed']
    assert json.loads((ROOT/'original_trajectory_identity.json').read_text())['all_30_segments_numeric_identical']
    settings=json.loads((ROOT/'hardware_config.json').read_text())
    package=Path(settings['package'])
    manifest_bytes=(package/'manifest.json').read_bytes()
    assert hashlib.sha256(manifest_bytes).hexdigest()==settings['manifest_sha256'], 'Package changed'
    manifest=json.loads(manifest_bytes)
    assert manifest['tracking_guard_deg']==[.5,.5,.5,.6,.5,.5]
    request=manifest['tracking_guard_request']
    assert hashlib.sha256(Path(request['path']).read_bytes()).hexdigest()==request['sha256']
    assert not Path(settings['claim_file']).exists(), 'This single-use request was already claimed'
    # Connecting telemetry does not authorize motion. The backend independently
    # requires fresh scene evidence, exact start state and an unused claim before
    # enabling the single-run button. At initial login that evidence is absent.
    deployment=json.loads((ROOT/'deployment.json').read_text())
    assert deployment['status']=='deployed_and_offline_verified'
    assert deployment['verification']['passed']
    opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
    current=json.load(opener.open('http://127.0.0.1:8767/api/snapshot',timeout=5))
    robot=current['robot'];state=robot['data']
    assert robot['fresh'] and state['stop_confirmed'] and not state['motion_active']
    assert state['queue']==0 and state['exec_id']==-1
    assert max(abs(a-b) for a,b in zip(state['q'],manifest['task_start_q']))<=math.radians(.05)
    assert (current.get('task') or {}).get('status') not in ('SUBMITTING','ACCEPTED','EXECUTING','CANCELING','UNKNOWN')
    print('任务二 Web UI：登录仅建立只读反馈；复位及现场核验完成前，启动按钮保持禁用。',flush=True)
    credentials=dict(username=input('SDK username [aubo]: ').strip() or 'aubo',
                     password=getpass.getpass('SDK password (not saved): '))
    try:
        # The existing launcher retains task records and refuses to close an
        # active/uncertain execution supervisor. Credentials use stdin only.
        subprocess.run([sys.executable,str(WEBUI/'start.py'),'--stop'],check=True)
        destination=WEBUI/'hardware_integration/hardware_config.json'
        backup=ROOT/('previous_ui_config_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')+'.json')
        with backup.open('xb') as file:file.write(destination.read_bytes())
        temporary=destination.with_name('task2-config.pending.json')
        with temporary.open('x') as file:json.dump(settings,file,ensure_ascii=False,indent=2)
        os.replace(temporary,destination)
        subprocess.run([sys.executable,str(WEBUI/'start.py'),'--auth-stdin','--open-browser'],
                       input=(json.dumps(credentials)+'\n').encode(),check=True)
        print('已连接任务二操作台：http://localhost:8767/；尚未提交运动。',flush=True)
    finally:
        credentials.clear()


if __name__=='__main__':
    try:main()
    except Exception as error:
        print(type(error).__name__+': '+str(error),file=sys.stderr)
        raise SystemExit(1)
