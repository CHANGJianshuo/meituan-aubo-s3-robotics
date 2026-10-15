#!/usr/bin/env python3
"""Package an accepted measured-scene run for the existing NUC path-buffer player.

No connection to hardware, no invented start transition, and no speed scaling.
The original full joint trajectories remain byte-for-byte available.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import uuid

import numpy as np
from scipy.interpolate import CubicSpline


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def export(run, inputs, output, adapter):
    if output.exists() and any(output.iterdir()):
        raise ValueError('Output must be new/empty')
    acceptance = json.loads((run/'acceptance.json').read_text())
    run_status = json.loads((run/'run.json').read_text())
    if not acceptance['passed'] or not run_status['success'] or not run_status['shutdown_clean']:
        raise ValueError('Simulation did not complete cleanly with independent acceptance')
    spec = importlib.util.spec_from_file_location('trajectory_math', adapter/'trajectory.py')
    math_ = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(math_)
    captured = json.loads((inputs/'captured_state.json').read_text())
    config = json.loads((run/'scenario.json').read_text())
    raw = (run/'commands.jsonl').read_bytes()
    replay_verified = False
    if config.get('replay_commands'):
        replay_verified = raw == Path(config['replay_commands']).read_bytes()
        if not replay_verified:
            raise ValueError('Replay changed the saved joint trajectory')
    commands = [json.loads(line) for line in raw.splitlines() if line.strip()]
    required = ['side_insert', 'hook_engage', 'test_lift', 'lift', 'transport',
                'pre_seat', 'seat_contact', 'unload', 'side_withdraw', 'retreat']
    if [c['primitive'] for c in commands] != required:
        raise ValueError('Unexpected task-one command sequence')
    if any(c['joint_names'] != math_.JOINTS or not c['simulation_only'] for c in commands):
        raise ValueError('Invalid simulation source')
    q0 = captured['samples'][-1]['q']
    start_error = max(abs(a-b) for a, b in zip(q0, commands[0]['points'][0]['positions']))
    if start_error > np.deg2rad(.01) or captured['max_joint_drift_deg'] > .02:
        raise ValueError('Trajectory is not rooted at the captured physical pose')
    common = dict(schema=1, plan_id=uuid.uuid4().hex, joint_names=math_.JOINTS,
                  units={'position': 'rad', 'velocity': 'rad/s', 'acceleration': 'rad/s^2', 'time': 's'},
                  source=str(run.resolve()), source_sha256=hashlib.sha256(raw).hexdigest(),
                  source_is_simulation=True, physical_execution_validated=False,
                  real_scene_collision_verified=False,
                  created_utc=datetime.now(timezone.utc).isoformat(), capture_unix=captured['captured_unix'])
    bundle = {**common, 'selection': 'task', 'segments': commands}
    stats = math_.validate(bundle, captured['limits'])
    cfg = captured['robot_configuration']
    for i in range(6):
        if np.deg2rad(stats['peak_velocity_deg_s'][i]) >= min(cfg['getJointMaxSpeeds'][i], cfg['getLimitJointMaxSpeeds'][i]):
            raise ValueError('Velocity exceeds active controller limit')
        if np.deg2rad(stats['peak_acceleration_deg_s2'][i]) >= min(cfg['getJointMaxAccelerations'][i], cfg['getLimitJointMaxAccelerations'][i]):
            raise ValueError('Acceleration exceeds active controller limit')
    output.mkdir(parents=True, exist_ok=True)
    for filename, selection in [('task_one_trajectory.json', 'task'), ('full_trajectory.json', 'full')]:
        value = {**bundle, 'selection': selection}
        (output/filename).write_text(json.dumps(value, indent=2)+'\n')
        math_.write_csv(output/filename.replace('.json', '.csv'), value)
    (output/'task_one_original_commands.jsonl').write_bytes(raw)
    # Retain exact uniform position samples that the unmodified player uploads.
    samples = []
    approximation = []
    for segment in commands:
        dt, dense = math_.dense(segment, .01)
        q = np.array([row[0] for row in dense])
        times = np.arange(len(q))*dt
        checkpoints = (times[:-1]+times[1:])/2
        original = np.array([math_.sample(segment, t)[0] for t in checkpoints])
        error = max(float(np.max(np.abs(CubicSpline(times, q, axis=0, bc_type=bc)(checkpoints)-original)))
                    for bc in ['natural', 'not-a-knot', 'clamped'])
        approximation.append(dict(primitive=segment['primitive'], max_midpoint_difference_deg=float(np.rad2deg(error))))
        samples.append(dict(primitive=segment['primitive'], interval_s=dt, positions_rad=q.tolist()))
    (output/'controller_position_samples.json').write_text(json.dumps(samples)+'\n')
    if max(r['max_midpoint_difference_deg'] for r in approximation) > .01:
        raise ValueError('Cubic resampling approximation is too large')
    for filename in ['trajectory.py', 'run_hardware.py', 'run.sh', 'test_runner.py']:
        shutil.copyfile(adapter/filename, output/filename)
    (output/'run.sh').chmod(0o755)
    meta = output/'metadata'
    meta.mkdir()
    for filename in ['captured_state.json', 'mount_identification.json', 'kinematics_reference.json',
                     'kinematics_validation.json', 'controller_matched_arm.urdf', 'mount_comparison.png']:
        shutil.copyfile(inputs/filename, meta/filename)
    for filename in ['trajectory_fk_reference.json', 'trajectory_kinematics_check.json',
                     'coordinate_consistency.json', 'software_validation.json']:
        if (inputs/filename).exists():shutil.copyfile(inputs/filename, meta/filename)
    for name, expected_hash in config['provenance'].items():
        source = Path(name)
        if sha(source) != expected_hash:
            raise ValueError('A frozen scene input changed: '+name)
        if source.name == 'result.yaml':
            shutil.copyfile(source, meta/'hand_eye_result.yaml')
        elif source.name == 'point_result.json':
            destination = 'T0_original_point.json' if source.parent.name.startswith('placement_area_') else 'red_handle_original_point.json'
            shutil.copyfile(source, meta/destination)
    capture = Path(captured['camera_capture_dir'])
    shutil.copyfile(capture/'color.png', meta/'physical_initial_image.png')
    shutil.copyfile(capture/'capture.json', meta/'physical_rgbd_capture.json')
    for filename in ['scenario.json', 'robot.urdf', 'robot.srdf', 'world.sdf', 'acceptance.json', 'result.json', 'run.json']:
        shutil.copyfile(run/filename, meta/filename)
    manifest = {**common, 'captured_limits': captured['limits'], 'expected_identity': captured['identity'],
                **captured['config'], 'nuc_ssh_host': captured['nuc_ssh_host'],
                'captured_q': q0, 'task_start_q': commands[0]['points'][0]['positions'],
                'expected_tcp_offset': cfg['getTcpOffset'], 'expected_payload': cfg['getPayload'],
                'operational_mode': captured['samples'][-1]['operational_mode'],
                'transition_duration_s': 0., 'transition_delta_deg': [0.]*6,
                'task_validation': stats, 'full_validation': stats,
                'execution_time_scale_default': 1., 'execution_text_confirmation_required': False,
                'frame_convention': config['frame_convention'],
                'tool_mount_yaw_deg': config['tool_mount_yaw_deg'],
                'captured_start_error_deg': float(np.rad2deg(start_error)),
                'identical_recorded_trajectory_replay_passed': replay_verified,
                'runtime_preconditions_satisfied_at_capture': False,
                'captured_mode': captured['samples'][-1]['robot_mode'],
                'captured_safety': captured['samples'][-1]['safety_mode'],
                'captured_controller_payload_kg': cfg['getPayload'][0],
                'interpolation_check': {'kind': 'offline cubic boundary-condition sensitivity at 10ms knots; not controller execution validation',
                                        'stages': approximation},
                'adapter_sources': {str((adapter/name).resolve()): sha(adapter/name)
                                    for name in ['trajectory.py', 'run_hardware.py', 'run.sh', 'test_runner.py']},
                'sha256': {name: sha(output/name) for name in ['task_one_trajectory.json', 'full_trajectory.json',
                                                              'task_one_original_commands.jsonl', 'controller_position_samples.json']}}
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    (output/'README.md').write_text(f'''# 本次现场场景的任务一轨迹

红色电池 → T0。仿真已通过，实际机械臂未执行。仿真起点来自本次控制柜读取，
没有附加旧姿态过渡段。`full` 与 `task` 都包含相同的本次 10 段轨迹。
原样回放验证通过：{replay_verified}。

- 参考坐标系：world = base_link；关节顺序 J1—J6，单位 rad、rad/s、rad/s²、s。
- 红色横梁顶面：{config['source_anchor']['base_xyz_m']} m，按用户指定固定。
- T0 表面：{config['targets']['T0']} m，原测量沿 base Z 下移 20 mm。
- 原 V1、44 mm 舌片；相对 ee_link 安装角 {config['tool_mount_yaw_deg']}°。
- 轨迹运动时间：{stats['nominal_duration_s']:.3f} s；默认倍率 1，不额外放慢。
- 每段上传和静止检查产生额外等待，CSV 的累计时间不包含这些等待。
- 初始关节角（度）：{[round(float(x), 6) for x in np.rad2deg(q0)]}。

文件包括完整路点 JSON、20 ms CSV、原始仿真命令、控制柜用的均匀位置采样，
以及标定关联、安装角识别、机器人模型和仿真验收证据。相机原始数据地址保存在 metadata。
控制柜路径缓冲使用位置重采样；保留了插值差异检查，未声称两种控制器逐时刻完全相同。

## NUC 入口

将整个目录放到 NUC 后运行 `./run.sh offline` 可离线检查。
`./run.sh inspect --trajectory task` 只读并核对状态。
`./run.sh execute --trajectory task --time-scale 1` 通过内置检查后会开始运动。

采集时控制柜为 {manifest['captured_mode']} / {manifest['captured_safety']}，
TCP 配置为零（规划器另行使用 V1 CAD TCP），负载配置为 {cfg['getPayload'][0]} kg。
这份文件包没有改变这些配置。实际工具负载仍需与现场一致；配置发生变化后应重新读取并更新匹配信息。
播放器会拒绝未使能、急停、反馈异常、起点变化或配置不匹配，不会自动上电、解锁或回零。
本次采集的关节位置不变，但断电状态的 J2 速度读数非零，实机执行前必须恢复有效的静止反馈。

仿真采用接触承托，没有强制挂载。安装方向通过图像识别；CAD TCP、质量、摩擦及环境模型仍有名义假设，
仿真验收不等于实机抓取已经验证。播放器的到点检查也不代替真实电池抓起和放稳的视觉检查。
''', encoding='utf-8')
    return dict(output=str(output), nominal_motion_seconds=stats['nominal_duration_s'],
                peak_joint_speed_deg_s=max(stats['peak_velocity_deg_s']),
                start_error_deg=manifest['captured_start_error_deg'], sha256=manifest['sha256'])


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run', type=Path, required=True)
    p.add_argument('--inputs', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--adapter', type=Path, default=Path('/home/operator/meituan/meituan_challenge_ws_with_gazebo_simulation/scripts/task_one_hardware'))
    args = p.parse_args()
    print(json.dumps(export(args.run, args.inputs, args.output, args.adapter), indent=2))
