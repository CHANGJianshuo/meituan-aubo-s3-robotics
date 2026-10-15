#!/usr/bin/env python3
"""Read-only sensitivity check of the exact positions uploaded by the player."""
import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np
from scipy.interpolate import CubicSpline

root=Path(__file__).resolve().parent
workspace=root.parents[2]
math_path=workspace/'outputs/webui_laptop_20261014/hardware_integration/player_settle_v2/trajectory.py'
spec=importlib.util.spec_from_file_location('trajectory_math',math_path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
source=root/'saved_trajectory/task_two_trajectory.json'
bundle=json.loads(source.read_text())
stored=json.loads((root/'saved_trajectory/controller_position_samples.json').read_text())
checks=[]
for index,(segment,upload) in enumerate(zip(bundle['segments'],stored),1):
    dt,values=module.dense(segment,.01)
    positions=np.array([value[0] for value in values])
    assert dt==upload['interval_s'] and positions.tolist()==upload['positions_rad']
    t=np.arange(len(values))*dt
    mid=(t[:-1]+t[1:])/2
    expected=np.array([module.sample(segment,float(time))[0] for time in mid])
    deviations={}
    for mode in ['natural','not-a-knot','clamped']:
        interpolated=CubicSpline(t,positions,bc_type=mode)(mid)
        deviations[mode]=float(np.rad2deg(abs(interpolated-expected).max()))
    checks.append(dict(stage=index,interval_s=dt,samples=len(values),
                       midpoint_max_deviation_deg=deviations))
maximum=max(value for check in checks for value in check['midpoint_max_deviation_deg'].values())
report=dict(passed=maximum<=.01,maximum_deviation_deg=maximum,threshold_deg=.01,
    uploaded_position_samples_identical=True,stages=checks,
    interpretation='Sensitivity analysis for three cubic interpolation boundary conditions; not a claim about undocumented controller interpolation internals.',
    source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    trajectory_math_sha256=hashlib.sha256(math_path.read_bytes()).hexdigest())
(root/'controller_interpolation_check.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='stages'},indent=2))
raise SystemExit(0 if report['passed'] else 1)
