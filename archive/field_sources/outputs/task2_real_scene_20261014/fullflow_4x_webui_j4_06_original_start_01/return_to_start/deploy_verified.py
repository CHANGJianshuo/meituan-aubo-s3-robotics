#!/usr/bin/env python3
"""Copy the reviewed package to a new NUC directory and run offline checks only."""
from datetime import datetime, timezone
import hashlib
import io
import json
from pathlib import Path
import shlex
import subprocess
import tarfile

root = Path(__file__).resolve().parent
workspace = root.parents[2]
previous = json.loads((root.parent/'deployment.json').read_text())
stamp = datetime.now(timezone.utc)
remote = '/home/robot/meituan_task2_return_original_'+stamp.strftime('%Y%m%dT%H%M%SZ')
transport = previous['transport_options']
ssh = ['ssh', '-T', '-o', 'ControlPath=/tmp/mtc_task2_ssh_20261014_2242/control',
       *transport, previous['ssh_host']]
source = root/'commissioning'
hashes = {str(p.relative_to(source)):hashlib.sha256(p.read_bytes()).hexdigest()
          for p in sorted(source.rglob('*')) if p.is_file() and '__pycache__' not in p.parts}
record = dict(started_utc=stamp.isoformat(), ssh_host=previous['ssh_host'],
              remote_directory=remote+'/commissioning', transport_options=transport,
              motion_commands_sent=0, status='deployment_incomplete', sha256=hashes)
record_path = root/'deployment.json'
with record_path.open('x') as f:
    json.dump(record, f, indent=2);f.write('\n')
try:
    subprocess.run([*ssh, shlex.join(['mkdir','-m','700',remote])], check=True, timeout=20)
    archive = io.BytesIO()
    with tarfile.open(fileobj=archive,mode='w:gz') as tar:
        for name in hashes:
            tar.add(source/name,arcname='commissioning/'+name,recursive=False)
    subprocess.run([*ssh, shlex.join(['tar','-xzf','-','-C',remote])],
                   input=archive.getvalue(), check=True, timeout=60)
    program = '''
import hashlib,json,pathlib,subprocess,sys
root=pathlib.Path(sys.argv[1]);expected=json.loads(sys.argv[2])
for name,digest in expected.items():
    assert hashlib.sha256((root/name).read_bytes()).hexdigest()==digest,name
packages=json.loads((root/'commissioning.json').read_text())['packages']
checks=[]
for item in packages:
    package=root/item['directory']
    result=subprocess.run([sys.executable,str(package/'run_hardware.py'),'offline','--root',str(package)],capture_output=True,text=True,timeout=90)
    checks.append(dict(package=item['directory'],returncode=result.returncode,stdout=result.stdout,stderr=result.stderr))
print(json.dumps(dict(file_hashes_verified=len(expected),offline_checks=checks,passed=all(c['returncode']==0 for c in checks),motion_commands_sent=0,python=sys.version)))
'''
    result = subprocess.run([*ssh, shlex.join([
        '/home/robot/miniforge3/envs/aubo-s3-hardware/bin/python','-c',program,
        record['remote_directory'],json.dumps(hashes)])], capture_output=True,text=True,
        check=True,timeout=120)
    record['verification']=json.loads(result.stdout)
    assert record['verification']['passed'], 'NUC offline validation failed'
    record['status']='deployed_and_offline_verified'
    record['completed_utc']=datetime.now(timezone.utc).isoformat()
except BaseException as error:
    record['error']=type(error).__name__+': '+str(error)
    raise
finally:
    record_path.write_text(json.dumps(record,indent=2)+'\n')
print(json.dumps(record['verification'],indent=2))
