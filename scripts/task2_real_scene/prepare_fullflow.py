#!/usr/bin/env python3
"""Build one replay-verified thirty-stage package with a pinned tracking policy."""
import argparse
import ast
from datetime import datetime, timezone
import importlib.util
import json
from pathlib import Path
import shutil
import uuid

import numpy as np
from scipy.interpolate import BPoly

from prepare_commissioning import sha


def prepare(saved, retiming, manifest_reference, adapter, output, motion_reference=None,
            j4_tracking_guard_deg=.5, tracking_request=None, all_tracking_guard_deg=None,
            tracking_monitor_only=False):
    validation = json.loads((saved/'validation.json').read_text())
    acceptance = json.loads((saved/'acceptance.json').read_text())
    timing = json.loads((retiming/'retiming_validation.json').read_text())
    input_commands = retiming/'candidate_commands_input.jsonl'
    source_path = saved/'task_two_trajectory.json'
    source = json.loads(source_path.read_text())
    if (not validation['identical_command_replay_passed'] or not acceptance['passed']
            or not acceptance.get('sequence_evidence',{}).get('passed')
            or not acceptance.get('fixed_orientation',{}).get('passed')
            or validation['sha256'][source_path.name] != sha(source_path)):
        raise ValueError('Complete exact replay and fixed-orientation acceptance are required')
    if timing['command_input_sha256'] != sha(input_commands):
        raise ValueError('Smoothed input commands changed')
    expected = [json.loads(s) for s in input_commands.read_text().splitlines()]
    actual = [json.loads(s) for s in (saved/'original_commands.jsonl').read_text().splitlines()]
    if actual != expected or source['segments'] != expected:
        raise ValueError('Accepted replay differs from the smoothed trajectory')
    phases = ['side_insert','hook_engage','test_lift','lift','transport',
              'pre_seat','seat_contact','unload','side_withdraw','retreat']
    order = source['order']
    if (order != 'ABC' or len(source['segments']) != 30
            or [(s['object'],s['primitive']) for s in source['segments']] !=
               [('battery_'+c,p) for c in order for p in phases]):
        raise ValueError('Expected the reviewed ABC thirty-stage sequence')
    reference_timing = None
    if motion_reference is None:
        if timing['requested_motion_ratio'] != .5 or abs(timing['actual_motion_ratio']-.5)>1e-9:
            raise ValueError('The trajectory has not achieved the requested half motion time')
    else:
        previous = json.loads(motion_reference.read_text())
        previous_seconds = previous.get('nominal_motion_s', 0)
        if (previous.get('status')!='PHYSICAL_FULLFLOW_SMOOTH_2X_PASSED'
                or previous.get('completed_motion_segments')!=30 or previous.get('order')!='ABC'
                or previous.get('original_uniform_tracking_guard_deg')!=.5
                or previous.get('final_stationary') is not True
                or previous.get('player_returned_normally') is not True
                or not np.isfinite(previous_seconds) or previous_seconds<=0):
            raise ValueError('A successful physical ABC run with the original guards is required')
        if abs(timing['planned_motion_s']-previous_seconds*.5)>1e-6:
            raise ValueError('The new path has not achieved half of the previous physical motion time')
        reference_timing=dict(path=str(motion_reference.resolve()),sha256=sha(motion_reference),
            previous_nominal_motion_s=previous_seconds,requested_ratio=.5,
            achieved_ratio=timing['planned_motion_s']/previous_seconds)
    caps=[timing[k] for k in ['maximum_velocity_deg_s','maximum_acceleration_deg_s2','maximum_jerk_deg_s3']]
    if not all(np.isfinite(x) and 0<x<=limit for x,limit in zip(caps,[13.,25.,200.])):
        raise ValueError('Full-flow derivative design limits changed')
    joint_caps=np.asarray(timing.get('maximum_joint_velocity_deg_s',[caps[0]]*6),dtype=float)
    if (joint_caps.shape!=(6,) or not np.all(np.isfinite(joint_caps))
            or np.any(joint_caps<=0) or np.any(joint_caps>caps[0])):
        raise ValueError('Invalid per-joint velocity limits')
    interval_checks=[]
    for index,segment in enumerate(actual,1):
        points=segment['points'];knots=np.array([p['t'] for p in points])
        polynomial=BPoly.from_derivatives(knots,np.array([
            [p[k] for k in ['positions','velocities','accelerations']] for p in points]))
        bounds=[float(np.rad2deg(np.abs(polynomial.derivative(n).c).max())) for n in (1,2,3)]
        if not all(np.isfinite(value) and value<=cap for value,cap in zip(bounds,caps)):
            raise ValueError('Exported polynomial exceeds derivative limits in stage '+str(index))
        joint_bounds=np.rad2deg(np.abs(polynomial.derivative(1).c).max(axis=(0,1)))
        if not np.all(np.isfinite(joint_bounds)) or np.any(joint_bounds>joint_caps):
            raise ValueError('Exported polynomial exceeds per-joint velocity limits in stage '+str(index))
        interval_checks.append(dict(stage=index,bernstein_derivative_bounds_deg=bounds,
                                   bernstein_joint_velocity_bounds_deg_s=joint_bounds.tolist()))
    player = (adapter/'run_hardware.py').read_text()
    if 'PRE_SEAT_J3_LAG_TOL' in player:
        raise ValueError('Use the unchanged player with uniform 0.5 degree tracking checks')
    guard_fields = dict(uniform_tracking_guard_deg=.5)
    declared = {node.targets[0].id:ast.literal_eval(node.value)
                for node in ast.parse(player).body if isinstance(node,ast.Assign)
                and len(node.targets)==1 and isinstance(node.targets[0],ast.Name)
                and node.targets[0].id in ('TRACKING_POLICY','TRACKING_LIMITS_DEG')}
    if all_tracking_guard_deg is not None and (all_tracking_guard_deg != .6 or j4_tracking_guard_deg != .5):
        raise ValueError('Unsupported or conflicting all-joint tracking policy')
    if tracking_monitor_only:
        if (all_tracking_guard_deg is not None or j4_tracking_guard_deg != .5
                or declared.get('TRACKING_POLICY') != 'task2_tracking_monitor_v1'
                or 'TRACKING_LIMITS_DEG' not in declared or declared['TRACKING_LIMITS_DEG'] is not None
                or tracking_request is None):
            raise ValueError('Monitor-only requires its exact player, explicit request and no numeric tracking selection')
        request=json.loads(tracking_request.read_text())
        if ('tracking_guard_deg' not in request or request['tracking_guard_deg'] is not None
                or request.get('tracking_guard_action') != 'record_only' or request.get('user_submission_only') is not True):
            raise ValueError('Monitor-only tracking request does not match the selected policy')
        guard_fields=dict(tracking_guard_deg=None,tracking_guard_policy='task2_tracking_monitor_v1',
            tracking_guard_action='record_only',tracking_guard_request=dict(path=str(tracking_request.resolve()),sha256=sha(tracking_request)))
    elif j4_tracking_guard_deg == .6 or all_tracking_guard_deg == .6:
        limits = [.6]*6 if all_tracking_guard_deg == .6 else [.5,.5,.5,.6,.5,.5]
        policy = 'task2_all_0p6_v1' if all_tracking_guard_deg == .6 else 'task2_j4_0p6_v1'
        if (declared.get('TRACKING_POLICY')!=policy
                or list(declared.get('TRACKING_LIMITS_DEG',[]))!=limits or tracking_request is None):
            raise ValueError('Tracking selection requires the matching player and pinned user request')
        request=json.loads(tracking_request.read_text())
        if request.get('tracking_guard_deg')!=limits or request.get('user_submission_only') is not True:
            raise ValueError('Tracking request does not match the selected limits and manual submission')
        guard_fields=dict(tracking_guard_deg=limits,tracking_guard_policy=policy,
            tracking_guard_request=dict(path=str(tracking_request.resolve()),sha256=sha(tracking_request)))
    elif j4_tracking_guard_deg != .5 or declared or tracking_request is not None:
        raise ValueError('Unsupported or unrequested tracking-policy change')
    reference = json.loads(manifest_reference.read_text())
    required = ['host','port','robot_name','expected_identity','expected_payload',
                'expected_tcp_offset','operational_mode','captured_limits']
    hardware = {k:reference[k] for k in required}
    spec = importlib.util.spec_from_file_location('trajectory_math',adapter/'trajectory.py')
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    scope='fullflow_2x' if reference_timing is None else 'fullflow_4x_webui'
    bundle = dict(source,plan_id=uuid.uuid4().hex,selection='full',commissioning_scope=scope)
    statistics = module.validate(bundle,hardware['captured_limits'])
    if abs(statistics['nominal_duration_s']-timing['planned_motion_s'])>1e-6:
        raise ValueError('Smoothed timing and replay duration disagree')
    for segment in source['segments']:
        for point in [segment['points'][0],segment['points'][-1]]:
            if point['velocities'] != [0.]*6 or point['accelerations'] != [0.]*6:
                raise ValueError('Every checked stop must have zero velocity and acceleration')
    output.mkdir(parents=True,exist_ok=False)
    slug='01_ABC_'+scope;package=output/slug;package.mkdir()
    for name in ['run_hardware.py','trajectory.py','test_runner.py','run.sh']:
        shutil.copyfile(adapter/name,package/name)
    (package/'run.sh').chmod(0o755)
    (package/'full_trajectory.json').write_text(json.dumps(bundle,indent=2)+'\n')
    manifest=dict(**hardware,plan_id=bundle['plan_id'],created_utc=datetime.now(timezone.utc).isoformat(),
        task_start_q=source['segments'][0]['points'][0]['positions'],full_validation=statistics,
        physical_execution_validated=False,**guard_fields,
        fullflow_without_manual_stage_requests=True,requested_motion_ratio=.5,
        motion_reference=reference_timing,maximum_joint_velocity_deg_s=joint_caps.tolist(),
        source_saved_trajectory_sha256=sha(source_path),retiming_sha256=sha(retiming/'retiming_validation.json'),
        sha256={'full_trajectory.json':sha(package/'full_trajectory.json')},
        adapter_sha256={n:sha(package/n) for n in ['run_hardware.py','trajectory.py','test_runner.py','run.sh']})
    (package/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    entry=dict(directory=slug,assignment=source['assignments'],scope=scope,source_stages=[1,30],
        start_q=manifest['task_start_q'],end_q=source['segments'][-1]['points'][-1]['positions'],
        motion_seconds=statistics['nominal_duration_s'],plan_id=bundle['plan_id'],manifest_sha256=sha(package/'manifest.json'))
    record=dict(order=order,packages=[entry],simulation_replay_verified=True,
        physical_execution_validated=False,motion_commands_sent=0,adapter_source=str(adapter.resolve()),
        preserved_stage_stationary_checks=True,motion_ratio=.5,motion_reference=reference_timing)
    record.update(guard_fields)
    (output/'commissioning.json').write_text(json.dumps(record,indent=2)+'\n')
    (output/'exported_polynomial_bounds.json').write_text(json.dumps(
        dict(passed=True,method='Conservative Bernstein bounds over every polynomial interval',
             caps_deg=caps,joint_velocity_caps_deg_s=joint_caps.tolist(),stages=interval_checks),indent=2)+'\n')
    comparison=('本次相同路径基线' if reference_timing is None else '上一轮已通过实机验证的完整流程')
    guard_description=('按用户请求，全部 30 段六轴跟踪误差仅记录，不按误差大小触发软件停止；控制柜保护未修改'
                       if tracking_monitor_only else
                       '按用户请求，全部 30 段六轴应用层跟踪阈值统一为 0.6°；控制柜保护未修改'
                       if all_tracking_guard_deg == .6 else
                       '统一 0.5° 跟踪保护' if j4_tracking_guard_deg==.5 else
                       '按用户请求将 J4 应用层跟踪阈值设为 0.6°，其他五轴仍为 0.5°；控制柜保护未修改')
    (output/'README.md').write_text('# 任务二：完整流程，平滑提速\n\n'
        '一次请求自动完成红→P1、黄→P2、蓝→P3 的全部 30 段，保持舌规朝向。'
        f'运动时间为{comparison}的一半；原停稳、接触与核验等待另计。'
        '各段起停速度和加速度为零，路径拟合偏差、速度、加速度及加加速度均已离线检查。\n\n'
        f'{guard_description}。保留身份/TCP/负载、起点、限位、独占锁和单次台账。'
        '本包尚未实机执行；最终现场结果另存。\n')
    print(json.dumps(record,indent=2));return record


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['saved','retiming','manifest-reference','adapter','output']:
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--motion-reference',type=Path,
                        help='Successful previous physical validation; requires half its nominal motion time')
    parser.add_argument('--j4-tracking-guard-deg',type=float,choices=[.5,.6],default=.5)
    parser.add_argument('--all-tracking-guard-deg',type=float,choices=[.6],default=None)
    parser.add_argument('--tracking-monitor-only',action='store_true')
    parser.add_argument('--tracking-request',type=Path,help='Pinned user-request record for the selected tracking variant')
    prepare(**vars(parser.parse_args()))
