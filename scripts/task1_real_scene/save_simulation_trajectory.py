#!/usr/bin/env python3
"""Save an accepted simulation's exact trajectory without a hardware launcher."""
import argparse
import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import uuid

import numpy as np


def save(run, output, math_path):
    status = json.loads((run/'run.json').read_text())
    acceptance = json.loads((run/'acceptance.json').read_text())
    scenario = json.loads((run/'scenario.json').read_text())
    if not status['success'] or not status['shutdown_clean'] or not acceptance['passed']:
        raise ValueError('A successful, independently accepted simulation and clean shutdown are required')
    raw = (run/'commands.jsonl').read_bytes()
    segments = [json.loads(line) for line in raw.splitlines() if line]
    phases = ['side_insert', 'hook_engage', 'test_lift', 'lift', 'transport',
              'pre_seat', 'seat_contact', 'unload', 'side_withdraw', 'retreat']
    order = scenario.get('requested_order', status.get('order', ''))
    count = 1 if scenario['task'] == 'basic' else 3
    if len(order) != count or len(set(order)) != count or any(c not in 'ABCD' for c in order):
        raise ValueError('Invalid task order')
    expected = [('battery_'+letter, phase) for letter in order for phase in phases]
    if [(s['object'], s['primitive']) for s in segments] != expected:
        raise ValueError('Incomplete task trajectory')
    replay_verified = bool(scenario.get('replay_commands'))
    if replay_verified and raw != Path(scenario['replay_commands']).read_bytes():
        raise ValueError('Recorded replay changed the original commands')
    events = [json.loads(line) for line in (run/'events.jsonl').read_text().splitlines() if line]
    replay_events = [e for e in events if e['event'] == 'replay_trajectory']
    replay_phases = {(e['object'], e['primitive']) for e in replay_events}
    executed_phases = [(e['object'], e['primitive']) for e in events if e['event'] == 'execute']
    # The FSM may fetch an unchanged command during both preview and execution.
    if executed_phases != expected:
        raise ValueError('Executed object/phase order differs from the requested task')
    if replay_verified and (replay_phases != set(expected) or
                            any(e['event'] in ('plan', 'replan_from_measured_start') for e in events)):
        raise ValueError('Replay must use every saved segment without replanning')
    spec = importlib.util.spec_from_file_location('trajectory_math', math_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if any(s['joint_names'] != module.JOINTS or not s['simulation_only'] for s in segments):
        raise ValueError('Unexpected trajectory source or joint order')
    q0 = [scenario['initial_joints'][name] for name in module.JOINTS]
    start_error = float(np.rad2deg(np.max(np.abs(np.array(q0)-segments[0]['points'][0]['positions']))))
    if start_error > .01:
        raise ValueError('Trajectory does not start at the declared captured posture')
    slots = ['T0'] if count == 1 else ['P1', 'P2', 'P3']
    assignments = [dict(object='battery_'+c, slot=s) for c, s in zip(order, slots)]
    bundle = dict(schema=1, plan_id=uuid.uuid4().hex, selection='task', joint_names=module.JOINTS,
                  task=scenario['task'], order=order, assignments=assignments,
                  units=dict(position='rad', velocity='rad/s', acceleration='rad/s^2', time='s'),
                  source=str(run.resolve()), source_is_simulation=True,
                  source_sha256=hashlib.sha256(raw).hexdigest(),
                  physical_execution_validated=False, real_scene_collision_verified=False,
                  accepted_for_robot_motion=False, execution_status='HOLD_PENDING_PHYSICAL_COORDINATE_VALIDATION',
                  segments=segments)
    statistics = module.validate(bundle, scenario.get('controller_joint_limits_rad'))
    output.mkdir(parents=True, exist_ok=False)
    stem = 'task_one_trajectory' if count == 1 else 'task_two_trajectory'
    (output/(stem+'.json')).write_text(json.dumps(bundle, indent=2)+'\n')
    if count == 1:
        module.write_csv(output/(stem+'.csv'), bundle)
    else:
        # Repeated primitives must remain attributable to a battery and slot.
        slot_for = {a['object']:a['slot'] for a in assignments}
        offset = 0.
        with (output/(stem+'.csv')).open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['time_s','object','slot','segment']+
                            [f'J{i}_{unit}' for unit in ('rad','rad_s','rad_s2') for i in range(1,7)])
            for index, segment in enumerate(segments):
                dt, dense = module.dense(segment, .02)
                for i,(q,v,a) in enumerate(dense):
                    if index and i == 0: continue
                    writer.writerow([format(offset+i*dt,'.9f'),segment['object'],slot_for[segment['object']],
                                     segment['primitive'],*q,*v,*a])
                offset += segment['points'][-1]['t']
    (output/'original_commands.jsonl').write_bytes(raw)
    for name in ['scenario.json', 'events.jsonl', 'acceptance.json', 'result.json', 'run.json']:
        (output/name).write_bytes((run/name).read_bytes())
    samples = []
    for segment in segments:
        dt, dense = module.dense(segment, .01)
        samples.append(dict(object=segment['object'], primitive=segment['primitive'], interval_s=dt,
                            positions_rad=[row[0] for row in dense]))
    (output/'controller_position_samples.json').write_text(json.dumps(samples)+'\n')
    report = dict(status=bundle['execution_status'], simulation_success=True,
                  identical_command_replay_passed=replay_verified, statistics=statistics,
                  initial_joint_error_deg=start_error, source_run=str(run.resolve()),
                  source_scene=str((run/'scenario.json').resolve()), time_scale=1.,
                  replay_segment_count=len(replay_phases), replay_command_load_count=len(replay_events),
                  trajectory_math_source=str(math_path.resolve()),
                  trajectory_math_sha256=hashlib.sha256(math_path.read_bytes()).hexdigest(),
                  physical_execution_validated=False, accepted_for_robot_motion=False,
                  hardware_launcher_included=False,
                  sha256={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in output.iterdir()})
    (output/'validation.json').write_text(json.dumps(report, indent=2)+'\n')
    (output/'README.md').write_text(
        '# 新场景的仿真轨迹\n\n'
        + '；'.join(a['object']+' → '+a['slot'] for a in assignments) + '。'
        'JSON 和原始命令保留完整路点、速度、加速度与时间；'
        'CSV 为采样查看文件，各段不大于 10 ms 的均匀位置采样单独保存（准确间隔见 interval_s）。默认时间倍率为 1。'
        '阶段验收与等待时序保存在 events.jsonl；CSV 的累计运动时间不含这些等待。\n\n'
        f'原样回放验证：{replay_verified}；运动时间约 {statistics["nominal_duration_s"]:.3f} 秒。\n\n'
        '**HOLD：现场坐标仍为候选，真实机械臂未执行本轨迹。** '
        '此目录保存轨迹数据，不含硬件启动器。仿真通过不等于实机坐标与间隙已经独立验证。\n', encoding='utf-8')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--trajectory-math', type=Path, default=Path('/home/operator/meituan/meituan_challenge_ws_with_gazebo_simulation/scripts/task_one_hardware/trajectory.py'))
    args = parser.parse_args()
    print(json.dumps(save(args.run, args.output, args.trajectory_math), indent=2))
