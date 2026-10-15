
import json,pathlib,hashlib
from datetime import datetime,timezone
active=[]
for p in pathlib.Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:args=(p/'cmdline').read_bytes().split(b'\0')
 except OSError:continue
 if b'execute' in args and any(pathlib.Path(a.decode(errors='replace')).name in ('remote_bridge.py','run_hardware.py') for a in args):active.append(dict(pid=p.name,args=[a.decode(errors='replace') for a in args]))
ledger=pathlib.Path('/home/robot/aubo_s3_nuc_smoke/hardware/attempts')
result=dict(created_utc=datetime.now(timezone.utc).isoformat(),active_execution_processes=active,ledgers={})
for plan in ['cde4f4eb32734cbb98c805f76884112d','134791c0efb04089854c756ab946dcd3']:
 p=ledger/(plan+'_full.json'); result['ledgers'][plan]=dict(path=str(p),exists=p.exists())
 if p.exists():result['ledgers'][plan].update(sha256=hashlib.sha256(p.read_bytes()).hexdigest(),content=json.loads(p.read_text()))
print(json.dumps(result))
