"""A full run must contain the exact accepted thirty-stage smooth trajectory."""
import importlib.util
import json
from pathlib import Path
import sys

import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts/task2_real_scene'))
spec=importlib.util.spec_from_file_location('prepare_fullflow',ROOT/'scripts/task2_real_scene/prepare_fullflow.py')
builder=importlib.util.module_from_spec(spec);spec.loader.exec_module(builder)
PHASES=['side_insert','hook_engage','test_lift','lift','transport','pre_seat',
        'seat_contact','unload','side_withdraw','retreat']


def fixture(tmp_path,fault=None):
    saved=tmp_path/'saved';saved.mkdir();retiming=tmp_path/'retiming';retiming.mkdir();adapter=tmp_path/'adapter';adapter.mkdir()
    segments=[dict(object='battery_'+c,primitive=p,points=[dict(t=t,positions=[0.]*6,
        velocities=[0.]*6,accelerations=[0.]*6) for t in (0.,1.)]) for c in 'ABC' for p in PHASES]
    if fault=='moving_stop':segments[0]['points'][-1]['velocities'][0]=.01
    if fault=='hidden_jerk_spike':
        segments[0]['points'].insert(1,dict(t=.000003,positions=[1e-12]+[0.]*5,
            velocities=[0.]*6,accelerations=[0.]*6))
    raw=''.join(json.dumps(s)+'\n' for s in segments)
    (retiming/'candidate_commands_input.jsonl').write_text(raw);(saved/'original_commands.jsonl').write_text(raw)
    bundle=dict(order='CAB' if fault=='wrong_order' else 'ABC',segments=segments,
        assignments=[dict(object='battery_'+c,slot='P'+str(i+1)) for i,c in enumerate('ABC')])
    path=saved/'task_two_trajectory.json';path.write_text(json.dumps(bundle))
    (saved/'validation.json').write_text(json.dumps(dict(identical_command_replay_passed=fault!='unverified_replay',sha256={path.name:builder.sha(path)})))
    (saved/'acceptance.json').write_text(json.dumps(dict(passed=True,sequence_evidence=dict(passed=True),fixed_orientation=dict(passed=True))))
    timing=dict(command_input_sha256=builder.sha(retiming/'candidate_commands_input.jsonl'),
                requested_motion_ratio=.5,actual_motion_ratio=.75 if fault=='wrong_speed' else .5,planned_motion_s=30.,
                maximum_velocity_deg_s=13.,maximum_acceleration_deg_s2=25.,maximum_jerk_deg_s3=200.)
    (retiming/'retiming_validation.json').write_text(json.dumps(timing))
    if fault=='changed_smoothed':(retiming/'candidate_commands_input.jsonl').write_text(raw+'\n')
    if fault=='changed_accepted':path.write_text(json.dumps(dict(bundle,order='CBA')))
    reference=tmp_path/'manifest.json';reference.write_text(json.dumps(dict(host='fixture',port=0,robot_name='fixture',
        expected_identity={},expected_payload=[],expected_tcp_offset=[],operational_mode='fixture',captured_limits=[[-3.]*6,[3.]*6])))
    for name in ['run_hardware.py','test_runner.py','run.sh']:(adapter/name).write_text('# fixture\n')
    if fault=='guard_exception':(adapter/'run_hardware.py').write_text('PRE_SEAT_J3_LAG_TOL=1\n')
    (adapter/'trajectory.py').write_text('def validate(bundle,limits):\n return {"nominal_duration_s":sum(s["points"][-1]["t"] for s in bundle["segments"])}\n')
    return dict(saved=saved,retiming=retiming,manifest_reference=reference,adapter=adapter,output=tmp_path/'commissioning')


def test_one_request_contains_all_thirty_stages_with_original_guards(tmp_path):
    args=fixture(tmp_path);report=builder.prepare(**args)
    assert len(report['packages'])==1
    entry=report['packages'][0];assert entry['source_stages']==[1,30]
    package=args['output']/entry['directory'];bundle=json.loads((package/'full_trajectory.json').read_text())
    assert len(bundle['segments'])==30 and bundle['selection']=='full'
    assert (package/'run_hardware.py').read_bytes()==(args['adapter']/'run_hardware.py').read_bytes()
    assert report['physical_execution_validated'] is False


@pytest.mark.parametrize('fault',['changed_smoothed','changed_accepted','wrong_order','wrong_speed',
                                  'moving_stop','guard_exception','unverified_replay','hidden_jerk_spike'])
def test_fullflow_rejects_unverified_changed_or_unsafe_bundle(tmp_path,fault):
    args=fixture(tmp_path,fault)
    with pytest.raises(ValueError):builder.prepare(**args)
    assert not args['output'].exists()


def previous_physical(tmp_path):
    path=tmp_path/'previous_physical.json'
    path.write_text(json.dumps(dict(status='PHYSICAL_FULLFLOW_SMOOTH_2X_PASSED',
        nominal_motion_s=60.,completed_motion_segments=30,order='ABC',
        original_uniform_tracking_guard_deg=.5,final_stationary=True,player_returned_normally=True)))
    return path


def test_shorter_geometry_uses_previous_physical_duration_as_reference(tmp_path):
    args=fixture(tmp_path);args['motion_reference']=previous_physical(tmp_path)
    timing=args['retiming']/'retiming_validation.json';d=json.loads(timing.read_text())
    d.update(requested_motion_ratio=.3,actual_motion_ratio=.3,maximum_joint_velocity_deg_s=[13.,13.,11.5,13.,13.,13.])
    timing.write_text(json.dumps(d))
    report=builder.prepare(**args)
    assert report['motion_reference']['achieved_ratio']==.5
    assert report['motion_reference']['sha256']==builder.sha(args['motion_reference'])
    package=args['output']/report['packages'][0]['directory']
    manifest=json.loads((package/'manifest.json').read_text())
    assert manifest['maximum_joint_velocity_deg_s'][2]==11.5


@pytest.mark.parametrize('change',[{'nominal_motion_s':59.},{'final_stationary':False},
    {'completed_motion_segments':29},{'original_uniform_tracking_guard_deg':.8},
    {'player_returned_normally':False}])
def test_rejects_wrong_timing_or_unverified_physical_reference(tmp_path,change):
    args=fixture(tmp_path);reference=previous_physical(tmp_path)
    d=json.loads(reference.read_text());d.update(change);reference.write_text(json.dumps(d))
    with pytest.raises(ValueError):builder.prepare(**args,motion_reference=reference)
    assert not args['output'].exists()


def test_independently_rechecks_per_joint_caps(tmp_path):
    args=fixture(tmp_path,'moving_stop')
    timing=args['retiming']/'retiming_validation.json';d=json.loads(timing.read_text())
    d['maximum_joint_velocity_deg_s']=[.1,13.,11.5,13.,13.,13.];timing.write_text(json.dumps(d))
    with pytest.raises(ValueError,match='per-joint velocity'):
        builder.prepare(**args)
    assert not args['output'].exists()


def test_requested_j4_policy_requires_matching_player_and_request(tmp_path):
    args=fixture(tmp_path)
    (args['adapter']/'run_hardware.py').write_text("TRACKING_POLICY='task2_j4_0p6_v1'\nTRACKING_LIMITS_DEG=(.5,.5,.5,.6,.5,.5)\n")
    request=tmp_path/'request.json';request.write_text(json.dumps(dict(
        tracking_guard_deg=[.5,.5,.5,.6,.5,.5],user_submission_only=True)))
    with pytest.raises(ValueError):builder.prepare(**args)
    with pytest.raises(ValueError):builder.prepare(**args,j4_tracking_guard_deg=.6)
    report=builder.prepare(**args,j4_tracking_guard_deg=.6,tracking_request=request)
    package=args['output']/report['packages'][0]['directory']
    manifest=json.loads((package/'manifest.json').read_text())
    assert 'uniform_tracking_guard_deg' not in manifest
    assert manifest['tracking_guard_deg']==[.5,.5,.5,.6,.5,.5]
    assert manifest['tracking_guard_request']['sha256']==builder.sha(request)


def test_original_player_cannot_be_labeled_as_j4_variant(tmp_path):
    args=fixture(tmp_path);request=tmp_path/'request.json'
    request.write_text(json.dumps(dict(tracking_guard_deg=[.5,.5,.5,.6,.5,.5],user_submission_only=True)))
    with pytest.raises(ValueError):builder.prepare(**args,j4_tracking_guard_deg=.6,tracking_request=request)
    assert not args['output'].exists()


def test_all_joint_policy_requires_matching_player_explicit_selection_and_request(tmp_path):
    args=fixture(tmp_path)
    (args['adapter']/'run_hardware.py').write_text("TRACKING_POLICY='task2_all_0p6_v1'\nTRACKING_LIMITS_DEG=(.6,.6,.6,.6,.6,.6)\n")
    request=tmp_path/'request.json'
    request.write_text(json.dumps(dict(tracking_guard_deg=[.6]*6,user_submission_only=True)))
    for kwargs in [{},{'all_tracking_guard_deg':.6},
                   {'all_tracking_guard_deg':.6,'j4_tracking_guard_deg':.6,'tracking_request':request}]:
        with pytest.raises(ValueError):builder.prepare(**args,**kwargs)
    report=builder.prepare(**args,all_tracking_guard_deg=.6,tracking_request=request)
    manifest=json.loads((args['output']/report['packages'][0]['directory']/'manifest.json').read_text())
    assert manifest['tracking_guard_policy']=='task2_all_0p6_v1'
    assert manifest['tracking_guard_deg']==[.6]*6
    assert manifest['tracking_guard_request']['sha256']==builder.sha(request)


@pytest.mark.parametrize('fault',['j4_player','wrong_limits','automatic_submission','unsupported_threshold'])
def test_all_joint_policy_rejects_mismatched_evidence(tmp_path,fault):
    args=fixture(tmp_path)
    policy='task2_j4_0p6_v1' if fault=='j4_player' else 'task2_all_0p6_v1'
    (args['adapter']/'run_hardware.py').write_text(f"TRACKING_POLICY={policy!r}\nTRACKING_LIMITS_DEG=(.6,.6,.6,.6,.6,.6)\n")
    request=tmp_path/'request.json'
    request.write_text(json.dumps(dict(tracking_guard_deg=[.5]*6 if fault=='wrong_limits' else [.6]*6,
                                      user_submission_only=fault!='automatic_submission')))
    with pytest.raises(ValueError):
        builder.prepare(**args,all_tracking_guard_deg=.8 if fault=='unsupported_threshold' else .6,tracking_request=request)
    assert not args['output'].exists()


def test_tracking_monitor_requires_explicit_request_and_preserves_thirty_stages(tmp_path):
    args=fixture(tmp_path)
    (args['adapter']/'run_hardware.py').write_text("TRACKING_POLICY='task2_tracking_monitor_v1'\nTRACKING_LIMITS_DEG=None\n")
    request=tmp_path/'request.json'
    request.write_text(json.dumps(dict(tracking_guard_deg=None,tracking_guard_action='record_only',user_submission_only=True)))
    for kwargs in [{},{'tracking_monitor_only':True},
                   {'tracking_monitor_only':True,'all_tracking_guard_deg':.6,'tracking_request':request}]:
        with pytest.raises(ValueError):builder.prepare(**args,**kwargs)
    result=builder.prepare(**args,tracking_monitor_only=True,tracking_request=request)
    package=args['output']/result['packages'][0]['directory']
    manifest=json.loads((package/'manifest.json').read_text())
    assert manifest['tracking_guard_deg'] is None
    assert manifest['tracking_guard_action']=='record_only'
    assert manifest['tracking_guard_request']['sha256']==builder.sha(request)
    assert len(json.loads((package/'full_trajectory.json').read_text())['segments'])==30


@pytest.mark.parametrize('fault',['numeric_player','missing_none','wrong_action','unrequested'])
def test_tracking_monitor_rejects_implicit_or_mismatched_request(tmp_path,fault):
    args=fixture(tmp_path)
    limits='(.6,.6,.6,.6,.6,.6)' if fault=='numeric_player' else 'None'
    (args['adapter']/'run_hardware.py').write_text(f"TRACKING_POLICY='task2_tracking_monitor_v1'\nTRACKING_LIMITS_DEG={limits}\n")
    data=dict(tracking_guard_deg=None,tracking_guard_action='record_only',user_submission_only=True)
    if fault=='missing_none':data.pop('tracking_guard_deg')
    if fault=='wrong_action':data['tracking_guard_action']='stop'
    if fault=='unrequested':data['user_submission_only']=False
    request=tmp_path/'request.json';request.write_text(json.dumps(data))
    with pytest.raises(ValueError):builder.prepare(**args,tracking_monitor_only=True,tracking_request=request)
    assert not args['output'].exists()
