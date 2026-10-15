import json
from pathlib import Path
import shutil
import sys
import time

root = Path(__file__).resolve().parent
camera = Path('/mnt/c/Users/chang/AppData/Local/mtc_camera_runtime/captures/session_20261012')
phase = sys.argv[1]
source = root / ('result.json' if phase == '090' else 'ready.json')
state = json.loads(source.read_text())
if phase != '090':
    assert state['stage'] == {'000': 'plus_05', '005': 'plus_30', '030': 'plus_60', '060': 'plus_90'}[phase]
name = f'j6_90_03_{phase}'
assert not (camera / 'request_capture.json').exists()
(camera / 'request_capture.json').write_text(json.dumps(dict(name=name, frames=1)))
deadline = time.monotonic() + 15
while not (camera / name / 'capture.json').exists():
    if time.monotonic() > deadline: raise RuntimeError('Capture timed out')
    time.sleep(.1)
shutil.copytree(camera / name, root / name, copy_function=shutil.copyfile)
(root / name / 'robot.json').write_text(json.dumps(state, indent=2))
print(root / name)
