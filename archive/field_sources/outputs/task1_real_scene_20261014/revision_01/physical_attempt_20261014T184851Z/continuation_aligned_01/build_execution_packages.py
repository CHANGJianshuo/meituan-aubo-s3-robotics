"""Package exact accepted continuation segments with the unchanged robot player."""
import hashlib
import json
from pathlib import Path
import shutil
import sys
import tarfile

HERE=Path(__file__).resolve().parent
ATT=HERE.parent
sys.path.insert(0,str(ATT))
import build_packages as builder

checks=json.loads((HERE/'continuation_checks.json').read_text())
negative=json.loads((HERE/'prealigned_guard_negative_test.json').read_text())
path=HERE/'saved_trajectory/task_one_trajectory.json'
assert checks['passed'] and negative['passed']
assert checks['candidate_sha256']==hashlib.sha256(path.read_bytes()).hexdigest()
bundle=json.loads(path.read_text())
builder.state=json.loads((ATT/'state_after_y_translation.json').read_text())
for name,segments,scope in [
 ('aligned_engage_testlift_package',bundle['segments'][1:3],
  'Exact newly simulated insertion and test lift only; stop for observed attachment check. Stationary simulation hold omitted.'),
 ('aligned_carry_place_package',bundle['segments'][3:],
  'Exact newly simulated lift, transport, seating and withdrawal; HOLD until physical test lift confirms stable attachment.')]:
 builder.package(name,{**bundle,'segments':segments,'continuation_trajectory_sha256':checks['candidate_sha256']},scope,[s['primitive'] for s in segments])
 p=ATT/name
 for file in ['continuation_checks.json','prealigned_guard_negative_test.json','alignment_reference.json']:
  shutil.copyfile(HERE/file,p/file)
 for file in ['validation.json','acceptance.json','run.json']:
  shutil.copyfile(HERE/'saved_trajectory'/file,p/('simulation_'+file))
 with tarfile.open(ATT/(name+'.tar.gz'),'w:gz') as archive:
  archive.add(p,arcname=name)
