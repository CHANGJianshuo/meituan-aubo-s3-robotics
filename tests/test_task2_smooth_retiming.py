"""Geometric bounds, complete sequence identity and C2 rest boundaries."""
import argparse
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest
from scipy.interpolate import BPoly

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('retimer',ROOT/'scripts/task1_real_scene/retime_successful_trajectory.py')
retimer = importlib.util.module_from_spec(spec);spec.loader.exec_module(retimer)
PHASES = ['side_insert','hook_engage','test_lift','lift','transport',
          'pre_seat','seat_contact','unload','side_withdraw','retreat']
JOINTS = ['shoulder_joint','upperArm_joint','foreArm_joint','wrist1_joint','wrist2_joint','wrist3_joint']


def fixture(tmp_path):
    source=tmp_path/'source.json';segments=[]
    for index,(letter,phase) in enumerate((l,p) for l in 'ABC' for p in PHASES):
        points=[dict(t=t,positions=[index*.001+offset]+[0.]*5,
                     velocities=[0.]*6,accelerations=[0.]*6) for t,offset in [(0.,0.),(4.,.001)]]
        segments.append(dict(object='battery_'+letter,primitive=phase,points=points))
    source.write_text(json.dumps(dict(order='ABC',segments=segments)))
    state=tmp_path/'state.json';state.write_text(json.dumps(dict(limits=[[-3.]*6,[3.]*6],
        robot_configuration={key:[10.]*6 for key in ['getJointMaxSpeeds','getLimitJointMaxSpeeds',
                                                   'getJointMaxAccelerations','getLimitJointMaxAccelerations']})))
    urdf=tmp_path/'arm.urdf';urdf.write_text('<robot>'+''.join(
        f'<joint name="{name}"><axis xyz="0 0 1"/><origin xyz="0 0 .01" rpy="0 0 0"/></joint>' for name in JOINTS)+'</robot>')
    # The reusable controller validator is already covered separately; test
    # this retimer's actual interpolation and limits without a deployed bundle.
    math_path=tmp_path/'trajectory.py';math_path.write_text(
        'def validate(bundle,limits):\n'
        ' return dict(nominal_duration_s=sum(s["points"][-1]["t"] for s in bundle["segments"]),'
        'peak_velocity_deg_s=[0.]*6,peak_acceleration_deg_s2=[0.]*6)\n')
    return argparse.Namespace(source=source,state=state,previous_log=None,output=tmp_path/'out',
        math=math_path,urdf=urdf,total_ratio=.5,motion_ratio=.5,velocity_deg_s=13.,
        acceleration_deg_s2=25.,jerk_deg_s3=200.,ramp_fraction=.08)


def test_refined_spline_respects_geometry_and_preserves_original_waypoints():
    knots=np.linspace(0,4,5);q=np.zeros((5,6));q[:,0]=[0,.03,.024,.057,.06]
    v=np.zeros_like(q);v[:,0]=np.gradient(q[:,0],knots)
    poly=BPoly.from_derivatives(knots,np.stack([q,v,np.zeros_like(q)],axis=1))
    def linear_fk(_model,values):
        t=np.tile(np.eye(4),(len(values),1,1));t[:,0,3]=values[:,0];return t
    smooth,joint,tool,refinement=retimer.bounded_spline(poly,knots,None,linear_fk)
    assert refinement>1 and joint<.02 and tool<.1
    np.testing.assert_allclose(smooth(knots),q,atol=1e-12)


def test_sample_clock_coalesces_nearly_coincident_waypoints():
    width=.08;duration=2.
    u=np.array([0.,.02,.0200001,.0200002,.08000001,.91999999,1.])
    knots=retimer.warp(u,width)[0]
    clock=retimer.sampling_clock(knots,0.,1.,duration,width)
    assert clock[0]==0 and clock[-1]==1
    assert np.diff(clock*duration).min()>=.001-1e-12


def test_arclength_removes_old_speed_variation_without_straightening_curve():
    knots=np.linspace(0,1,17)
    q=np.zeros((len(knots),6));v=q.copy();a=q.copy()
    q[:,0]=.1*knots**3;q[:,1]=.02*knots**6
    v[:,0]=.3*knots**2;v[:,1]=.12*knots**5
    a[:,0]=.6*knots;a[:,1]=.6*knots**4
    original=BPoly.from_derivatives(knots,np.stack([q,v,a],axis=1))
    def fk(_model,values):
        t=np.tile(np.eye(4),(len(values),1,1));t[:,:2,3]=values[:,:2];return t
    smooth,grid,joint,tool,_=retimer.arclength_spline(original,0.,1.,[13.]*6,None,fk)
    np.testing.assert_allclose(smooth([0.,1.]),q[[0,-1]],atol=1e-12)
    samples=smooth(np.linspace(0,1,1001))
    # The second coordinate remains curved, rather than a joint-space chord.
    assert np.abs(samples[:,1]-2*samples[:,0]**2).max()<1e-5
    speed=np.linalg.norm(smooth(np.linspace(.01,.99,1001),nu=1),axis=1)
    assert speed.max()/speed.min()<1.01
    assert joint<.02 and tool<.1


@pytest.mark.parametrize('parameterization',['arclength','adaptive'])
def test_arclength_full_sequence_keeps_checked_stops(tmp_path,parameterization):
    args=fixture(tmp_path);args.path_parameterization=parameterization
    args.joint_velocity_deg_s=[13.,13.,11.5,13.,13.,13.]
    retimer.retime(args)
    rows=[json.loads(line) for line in (args.output/'candidate_commands_input.jsonl').read_text().splitlines()]
    assert sum(s['points'][-1]['t'] for s in rows)==pytest.approx(60.)
    for segment in rows:
        assert segment['points'][0]['velocities']==segment['points'][-1]['velocities']==[0.]*6
        assert segment['points'][0]['accelerations']==segment['points'][-1]['accelerations']==[0.]*6
    report=json.loads((args.output/'retiming_validation.json').read_text())
    assert report['path_parameterization']==parameterization


def test_full_sequence_retimes_to_half_with_zero_endpoint_derivatives(tmp_path):
    args=fixture(tmp_path);original=json.loads(args.source.read_text())
    retimer.retime(args)
    rows=[json.loads(s) for s in (args.output/'candidate_commands_input.jsonl').read_text().splitlines()]
    assert len(rows)==30
    assert sum(s['points'][-1]['t'] for s in rows)==pytest.approx(60.)
    for index,(source,result) in enumerate(zip(original['segments'],rows)):
        assert result['object']==source['object'] and result['primitive']==source['primitive']
        for old,new in zip([source['points'][0],source['points'][-1]],
                           [result['points'][0],result['points'][-1]]):
            np.testing.assert_allclose(new['positions'],old['positions'],atol=1e-12)
            assert new['velocities']==new['accelerations']==[0.]*6
        if index:np.testing.assert_allclose(rows[index-1]['points'][-1]['positions'],result['points'][0]['positions'],atol=1e-12)
        points=result['points'];times=np.array([p['t'] for p in points]);assert np.all(np.diff((times*1e9).astype('int64'))>0)
        poly=BPoly.from_derivatives(times,np.array([[p[k] for k in ['positions','velocities','accelerations']] for p in points]))
        probe=np.linspace(0,times[-1],1001)
        for derivative,cap in [(1,13.),(2,25.),(3,200.)]:
            assert np.rad2deg(np.abs(poly(probe,nu=derivative)).max())<=cap
            assert np.rad2deg(np.abs(poly.derivative(derivative).c).max())<=cap


def test_joint_velocity_cap_controls_timing_and_exported_polynomial(tmp_path):
    args=fixture(tmp_path)
    args.joint_velocity_deg_s=[.04,13.,11.5,13.,13.,13.]
    # Only joint one moves in this fixture. The cap makes the requested half
    # duration infeasible; a longer requested duration must stay under that cap.
    with pytest.raises(ValueError,match='bounded minimum'):
        retimer.retime(args)
    assert not args.output.exists()
    args.motion_ratio=1.
    retimer.retime(args)
    report=json.loads((args.output/'retiming_validation.json').read_text())
    assert report['maximum_joint_velocity_deg_s']==args.joint_velocity_deg_s
    assert all(p['bernstein_joint_velocity_bounds_deg_s'][0]<=.04
               for p in report['part_validation'])


@pytest.mark.parametrize('caps',[[13.]*5,[13.]*5+[float('nan')],[13.]*5+[14.],[13.]*5+[0.]])
def test_invalid_joint_caps_fail_before_emitting_commands(tmp_path,caps):
    args=fixture(tmp_path);args.joint_velocity_deg_s=caps
    with pytest.raises(ValueError,match='joint velocity caps'):
        retimer.retime(args)
    assert not args.output.exists()


@pytest.mark.parametrize('fault',['missing_last','wrong_object','infeasible_speed','bad_ramp','bad_cap'])
def test_incomplete_or_infeasible_retiming_does_not_emit_commands(tmp_path,fault):
    args=fixture(tmp_path)
    if fault in ('missing_last','wrong_object'):
        source=json.loads(args.source.read_text())
        if fault=='missing_last':source['segments'].pop()
        else:source['segments'][10]['object']='battery_A'
        args.source.write_text(json.dumps(source))
    elif fault=='infeasible_speed':args.motion_ratio=.001
    elif fault=='bad_ramp':args.ramp_fraction=0
    elif fault=='bad_cap':args.jerk_deg_s3=float('nan')
    with pytest.raises(ValueError):retimer.retime(args)
    assert not (args.output/'candidate_commands_input.jsonl').exists()
