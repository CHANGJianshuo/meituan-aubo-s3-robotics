"""Credential-stdin maintenance: read-only inspect or UI startup, never submit."""
import argparse,json,subprocess,sys,termios
from pathlib import Path
HERE=Path(__file__).resolve().parent
parser=argparse.ArgumentParser()
parser.add_argument('action',choices=['inspect','start'])
parser.add_argument('package',choices=['resume','repeat'])
args=parser.parse_args()
old=None
try:
    if sys.stdin.isatty():
        old=termios.tcgetattr(sys.stdin.fileno());new=old.copy();new[3]&=~termios.ECHO
        termios.tcsetattr(sys.stdin.fileno(),termios.TCSANOW,new)
    print('Awaiting credentials on stdin (not echoed or saved)',flush=True)
    credentials=json.loads(sys.stdin.readline())
finally:
    if old is not None:termios.tcsetattr(sys.stdin.fileno(),termios.TCSANOW,old)
assert set(credentials)=={'username','password'}
if args.action=='inspect':
    remote='/home/robot/meituan_webui_verified_20261014/'+HERE.name+'/'+args.package+'_package'
    cmd=['ssh','-T','-o','BatchMode=yes','-o','ConnectTimeout=8','-o','HostKeyAlias=192.0.2.11',
         '-o','StrictHostKeyChecking=yes','robot@192.0.2.10',
         '/home/robot/miniforge3/envs/aubo-s3-hardware/bin/python '+remote+'/run_hardware.py inspect --root '+remote+' --auth-stdin']
else:
    cmd=[sys.executable,str(HERE.parents[1]/'start.py'),'--auth-stdin']
result=subprocess.run(cmd,input=json.dumps(credentials)+'\n',text=True,capture_output=True,timeout=65)
credentials.clear()
(HERE/(args.package+'-'+args.action+'.txt')).write_text(result.stdout+result.stderr)
print(result.stdout,flush=True)
if result.stderr:print(result.stderr,file=sys.stderr,flush=True)
raise SystemExit(result.returncode)
