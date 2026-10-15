"""Sequence export must retain object identity as well as each repeated phase."""
import importlib.util
import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('save_simulation_trajectory', ROOT/'scripts/task1_real_scene/save_simulation_trajectory.py')
exporter = importlib.util.module_from_spec(spec); spec.loader.exec_module(exporter)
PHASES = ['side_insert','hook_engage','test_lift','lift','transport',
          'pre_seat','seat_contact','unload','side_withdraw','retreat']
JOINTS = ['shoulder_joint','upperArm_joint','foreArm_joint','wrist1_joint','wrist2_joint','wrist3_joint']


def fixture(tmp_path, monkeypatch, order='ABC'):
    run=tmp_path/'run';run.mkdir()
    phases=[(f'battery_{letter}',phase) for letter in order for phase in PHASES]
    commands=[dict(object=o,primitive=p,simulation_only=True,joint_names=JOINTS,
                   points=[dict(t=t,positions=[0.]*6,velocities=[0.]*6,accelerations=[0.]*6) for t in (0.,1.)])
              for o,p in phases]
    raw=''.join(json.dumps(c)+'\n' for c in commands)
    source=tmp_path/'source.jsonl';source.write_text(raw)
    (run/'commands.jsonl').write_text(raw)
    events=[dict(event=kind,object=o,primitive=p) for o,p in phases for kind in ('replay_trajectory','execute')]
    (run/'events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in events))
    cfg=dict(task='basic' if len(order)==1 else 'sequence',requested_order=order,
             replay_commands=str(source),initial_joints=dict.fromkeys(JOINTS,0.),controller_joint_limits_rad=[[-3.]*6,[3.]*6])
    for name,data in [('scenario',cfg),('run',dict(success=True,shutdown_clean=True,order=order)),
                      ('acceptance',dict(passed=True)),('result',dict(success=True))]:
        (run/(name+'.json')).write_text(json.dumps(data))
    # Only the external interpolation math is replaced; command/order/replay
    # validation, byte preservation and export logic run without mocking.
    math_path=tmp_path/'trajectory.py'
    math_path.write_text(f'JOINTS={JOINTS!r}\n'
        'def validate(bundle, limits): return {"nominal_duration_s":float(len(bundle["segments"]))}\n'
        'def write_csv(path, bundle): path.write_text("test interpolation adapter\\n")\n'
        'def dense(segment, dt): return dt, [([0.]*6,[0.]*6,[0.]*6)]\n')
    return run,math_path


@pytest.mark.parametrize('order,stem',[('ABC','task_two_trajectory'),('A','task_one_trajectory')])
def test_export_preserves_full_object_sequence_and_original_commands(tmp_path,monkeypatch,order,stem):
    run,math_path=fixture(tmp_path,monkeypatch,order)
    report=exporter.save(run,tmp_path/'saved',math_path)
    bundle=json.loads((tmp_path/'saved'/(stem+'.json')).read_text())
    assert bundle['order']==order
    assert len(bundle['segments'])==10*len(order)
    assert report['replay_segment_count']==10*len(order)
    assert (tmp_path/'saved/original_commands.jsonl').read_bytes()==(run/'commands.jsonl').read_bytes()
    assert bundle['accepted_for_robot_motion'] is False


@pytest.mark.parametrize('fault',['wrong_object','missing_last_object','reordered_execute','replanned'])
def test_export_rejects_wrong_or_incomplete_repeated_phases(tmp_path,monkeypatch,fault):
    run,math_path=fixture(tmp_path,monkeypatch)
    if fault in ('wrong_object','missing_last_object'):
        p=run/'commands.jsonl';rows=[json.loads(s) for s in p.read_text().splitlines()]
        if fault=='wrong_object':rows[10]['object']='battery_A'
        else:rows=rows[:20]
    else:
        p=run/'events.jsonl';rows=[json.loads(s) for s in p.read_text().splitlines()]
        if fault=='reordered_execute':rows[1],rows[21]=rows[21],rows[1]
        else:rows.append(dict(event='replan_from_measured_start',object='battery_A'))
    p.write_text(''.join(json.dumps(x)+'\n' for x in rows))
    with pytest.raises(ValueError):exporter.save(run,tmp_path/'saved',math_path)
    assert not (tmp_path/'saved').exists()
