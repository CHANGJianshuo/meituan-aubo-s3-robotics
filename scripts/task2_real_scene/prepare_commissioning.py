#!/usr/bin/env python3
"""Split a replay-verified sequence into independently inspectable field stages.

Uses the preserved task-one player, without connecting to or moving a robot.
Approach, insertion/trial lift and carry/place are separate single-use packages.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import uuid


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def prepare(saved, manifest_reference, adapter, output):
    validation = json.loads((saved/'validation.json').read_text())
    acceptance = json.loads((saved/'acceptance.json').read_text())
    source_path = saved/'task_two_trajectory.json'
    if (not validation['identical_command_replay_passed'] or not acceptance['passed']
            or not acceptance.get('sequence_evidence', {}).get('passed')
            or not acceptance.get('fixed_orientation', {}).get('passed')
            or validation['sha256'][source_path.name] != sha(source_path)):
        raise ValueError('Exact replay, sequence, fixed-orientation and file identity checks must pass')
    source = json.loads(source_path.read_text())
    reference = json.loads(manifest_reference.read_text())
    required = ['host','port','robot_name','expected_identity','expected_payload',
                'expected_tcp_offset','operational_mode','captured_limits']
    hardware = {key:reference[key] for key in required}
    spec = importlib.util.spec_from_file_location('trajectory_math', adapter/'trajectory.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    if len(source['segments']) != 30:
        raise ValueError('Expected all thirty sequence stages')
    # No task-one-specific tracking exception is transferred to new task paths.
    player = (adapter/'run_hardware.py').read_text()
    if 'PRE_SEAT_J3_LAG_TOL' in player:
        raise ValueError('Use the task-one settle-v2 player with the uniform 0.5 degree tracking guard')
    output.mkdir(parents=True, exist_ok=False)
    parent = uuid.uuid4().hex
    packages = []
    for index, assignment in enumerate(source['assignments']):
        letter = assignment['object'].removeprefix('battery_')
        for label, lo, hi in [('approach',0,1), ('insert_trial_lift',1,3), ('carry_place',3,10)]:
            segments = source['segments'][index*10+lo:index*10+hi]
            name = f'{index+1:02d}_{letter}_{assignment["slot"]}_{label}'
            root = output/name; root.mkdir()
            bundle = {**source, 'plan_id':uuid.uuid4().hex, 'parent_plan_id':parent,
                      'selection':'full', 'segments':segments,
                      'commissioning_scope':label, 'source_stage_range':[index*10+lo+1,index*10+hi]}
            statistics = module.validate(bundle, hardware['captured_limits'])
            for filename in ['run_hardware.py','trajectory.py','test_runner.py','run.sh']:
                shutil.copyfile(adapter/filename, root/filename)
            (root/'run.sh').chmod(0o755)
            (root/'full_trajectory.json').write_text(json.dumps(bundle,indent=2)+'\n')
            manifest = dict(**hardware, plan_id=bundle['plan_id'], parent_plan_id=parent,
                created_utc=datetime.now(timezone.utc).isoformat(), commissioning_scope=label,
                assignment=assignment, source_saved_trajectory_sha256=sha(source_path),
                task_start_q=segments[0]['points'][0]['positions'],
                full_validation=statistics, physical_execution_validated=False,
                physical_alignment_pending=True, uniform_tracking_guard_deg=.5,
                source_manifest=str(manifest_reference.resolve()), source_manifest_sha256=sha(manifest_reference),
                sha256={'full_trajectory.json':sha(root/'full_trajectory.json')},
                adapter_sha256={n:sha(root/n) for n in ['run_hardware.py','trajectory.py','test_runner.py','run.sh']})
            (root/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
            packages.append(dict(directory=name, assignment=assignment, scope=label,
                                 source_stages=bundle['source_stage_range'],
                                 start_q=segments[0]['points'][0]['positions'],
                                 end_q=segments[-1]['points'][-1]['positions'],
                                 motion_seconds=statistics['nominal_duration_s'],
                                 plan_id=bundle['plan_id'], manifest_sha256=sha(root/'manifest.json')))
    record = dict(parent_plan_id=parent, order=source['order'], packages=packages,
                  simulation_replay_verified=True, physical_execution_validated=False,
                  motion_commands_sent=0, adapter_source=str(adapter.resolve()))
    (output/'commissioning.json').write_text(json.dumps(record,indent=2)+'\n')
    (output/'README.md').write_text(
        '# 任务二分段现场调试包\n\n'
        '红 → P1、黄 → P2、蓝 → P3；固定舌规朝向。整条轨迹已原样仿真回放，实机尚未执行。\n\n'
        '每块分为接近、插入和试提、搬运放置三组。每个目录单独校验起点并只允许一次执行；'
        '没有自动串联九组的启动器。先核对接近终点的舌片与把手间隙，再确认试提，最后核对落位。'
        '中途人工调整后须重新读取姿态和规划衔接，不能跳过起点检查。\n\n'
        '在 NUC 的对应目录，`./run.sh offline` 只检查文件；`./run.sh inspect` 只读控制柜。'
        '`./run.sh execute` 会执行该目录中的动作；凭据由终端隐藏输入，不写入文件。'
        '运行前逐项核对 `commissioning.json` 的对象、目标及起点。\n\n'
        '复用任务一 settle-v2 播放器，保留统一 0.5° 跟踪误差检查、连续停稳、身份/TCP/负载、'
        '限位、独占锁和单次执行台账。任务一特定路径的 J3 放宽策略没有移植。\n', encoding='utf-8')
    print(json.dumps(record,indent=2))
    return record


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['saved','manifest-reference','adapter','output']:
        p.add_argument('--'+name,type=Path,required=True)
    prepare(**vars(p.parse_args()))
