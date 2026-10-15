#!/usr/bin/env python3
"""Read completed NUC logs and the single-use ledger; never send robot commands."""
from collections import Counter
import hashlib
import io
import json
from pathlib import Path
import shlex
import subprocess
import tarfile

root=Path(__file__).resolve().parent
deployment=json.loads((root/'deployment.json').read_text())
remote=deployment['remote_directory']
ssh=['ssh','-T','-o','ControlPath=/tmp/mtc_task2_ssh_20261014_2242/control',
     *deployment['transport_options'],deployment['ssh_host']]
program='''
from collections import Counter
import hashlib,json,pathlib,sys
root=pathlib.Path(sys.argv[1])
packages=json.loads((root/'commissioning.json').read_text())['packages']
report=dict(package_logs={},matching_execution_processes=[],execution_log_counts={},total_motion_intents=0,ledgers={})
for entry in packages:
    name=entry['directory'];logs=[];count=0
    for path in sorted((root/name/'logs').glob('*.jsonl')):
        rows=[json.loads(line) for line in path.read_text().splitlines()]
        events=Counter(row['event'] for row in rows)
        logs.append(dict(path=str(path),relative_path=str(path.relative_to(root)),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),event_counts=events,begin=[row for row in rows if row['event']=='begin']))
        count+=bool(events['move_intent']);report['total_motion_intents']+=events['move_intent']
        for row in rows:
            if row['event']=='attempt_reserved':
                ledger=pathlib.Path(row['path'])
                report['ledgers'][entry['plan_id']]=dict(path=str(ledger),sha256=hashlib.sha256(ledger.read_bytes()).hexdigest(),content=json.loads(ledger.read_text()))
    report['package_logs'][name]=logs;report['execution_log_counts'][name]=count
for folder in pathlib.Path('/proc').glob('[0-9]*'):
    try:
        argv=(folder/'cmdline').read_bytes().decode().split('\\0')
        if any(arg.endswith(('/remote_bridge.py','/run_hardware.py')) for arg in argv) and any(str(root) in arg for arg in argv):
            report['matching_execution_processes'].append(dict(pid=int(folder.name),argv=argv))
    except (OSError,UnicodeError):
        pass
print(json.dumps(report))
'''
result=subprocess.run([*ssh,shlex.join(['/home/robot/miniforge3/envs/aubo-s3-hardware/bin/python','-c',program,remote])],capture_output=True,text=True,check=True,timeout=30)
audit=json.loads(result.stdout)
assert not audit['matching_execution_processes'], 'Execution process still active'
paths=[item['relative_path'] for logs in audit['package_logs'].values() for item in logs if item['event_counts'].get('move_intent',0)]
assert paths, 'No motion log to archive'
archive=subprocess.run([*ssh,shlex.join(['tar','-czf','-','-C',remote,*paths])],capture_output=True,check=True,timeout=60).stdout
with tarfile.open(fileobj=io.BytesIO(archive),mode='r:gz') as tar:
    for package,logs in audit['package_logs'].items():
        for item in logs:
            if not item['event_counts'].get('move_intent',0):continue
            member=tar.getmember(item['relative_path'])
            assert member.isfile()
            data=tar.extractfile(member).read()
            assert hashlib.sha256(data).hexdigest()==item['sha256']
            folder=root/'nuc_logs'/package;folder.mkdir(parents=True,exist_ok=True)
            with (folder/Path(item['path']).name).open('xb') as f:f.write(data)
with (root/'final_remote_audit.json').open('x') as f:
    json.dump(audit,f,indent=2);f.write('\n')
print(json.dumps(audit,indent=2))
