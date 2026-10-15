"""Package reviewed stage boundaries for supervised commissioning; no hardware I/O."""
import hashlib
import json
from pathlib import Path
import shutil
import sys
import uuid

HERE = Path(__file__).resolve().parent
REV = HERE.parent
ADAPTER = Path('/home/operator/meituan/meituan_challenge_ws_with_gazebo_simulation/scripts/task_one_hardware')
sys.path.insert(0, str(ADAPTER))
from trajectory import validate, write_csv, RAD

state = json.loads((HERE/'state_running.json').read_text())
source_path = REV/'saved_trajectory/task_one_trajectory.json'
source = json.loads(source_path.read_text())
verification = json.loads((HERE/'alignment/verification.json').read_text())
assert verification['passed']
assert verification['candidate_sha256'] == hashlib.sha256((HERE/'alignment/candidate.json').read_bytes()).hexdigest()
assert json.loads((REV/'replay_red_01/run.json').read_text())['success']
assert json.loads((REV/'replay_red_01/acceptance.json').read_text())['passed']
assert source['source_sha256'] == hashlib.sha256((REV/'replay_red_01/commands.jsonl').read_bytes()).hexdigest()


def package(name, bundle, scope, expected_primitives):
    out = HERE/name
    out.mkdir()
    assert [s['primitive'] for s in bundle['segments']] == expected_primitives
    bundle = {**bundle, 'plan_id': uuid.uuid4().hex, 'parent_plan_id': source['plan_id'],
              'physical_execution_validated': False, 'commissioning_scope': scope,
              'authorization': 'User explicitly requested physical execution with onsite supervision',
              'source_saved_trajectory_sha256': hashlib.sha256(source_path.read_bytes()).hexdigest()}
    stats = validate(bundle, state['limits'])
    cfg = state['robot_configuration']
    for i in range(6):
        assert stats['peak_velocity_deg_s'][i]*RAD < min(cfg['getJointMaxSpeeds'][i], cfg['getLimitJointMaxSpeeds'][i])
        assert stats['peak_acceleration_deg_s2'][i]*RAD < min(cfg['getJointMaxAccelerations'][i], cfg['getLimitJointMaxAccelerations'][i])
    for filename, selection in [('task_one_trajectory.json', 'task'), ('full_trajectory.json', 'full')]:
        value = {**bundle, 'selection': selection}
        (out/filename).write_text(json.dumps(value, indent=2)+'\n')
        write_csv(out/filename.replace('.json', '.csv'), value)
    manifest = dict(plan_id=bundle['plan_id'], captured_limits=state['limits'],
                    expected_identity=state['identity'], **state['config'],
                    expected_tcp_offset=cfg['getTcpOffset'], expected_payload=cfg['getPayload'],
                    operational_mode=state['samples'][-1]['operational_mode'],
                    captured_reference_q=state['samples'][-1]['q'],
                    task_start_q=bundle['segments'][0]['points'][0]['positions'],
                    physical_execution_validated=False, commissioning_scope=scope,
                    geometry_validation_pending=True, unchanged_player_checks=True,
                    task_validation=stats, full_validation=stats,
                    source_saved_trajectory_sha256=bundle['source_saved_trajectory_sha256'],
                    sha256={name:hashlib.sha256((out/name).read_bytes()).hexdigest()
                            for name in ['task_one_trajectory.json', 'full_trajectory.json']})
    for filename in ['trajectory.py', 'run_hardware.py', 'test_runner.py', 'run.sh']:
        shutil.copyfile(ADAPTER/filename, out/filename)
    (out/'run.sh').chmod(0o755)
    manifest['adapter_sha256'] = {name:hashlib.sha256((out/name).read_bytes()).hexdigest()
                                 for name in ['trajectory.py', 'run_hardware.py', 'test_runner.py', 'run.sh']}
    (out/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    (out/'README.md').write_text('# 分段实机调试\n\n范围：'+scope+'。\n\n'
        '复用原播放器，起点、限位、速度、配置、运动状态和执行中跟踪检查不变；没有自动重试。'
        '接近动作后需要独立核对现场插入间隙，尚未完成整项实机任务。完整已保存仿真轨迹未修改。\n')
    print(json.dumps({'package': str(out), 'stages': expected_primitives, 'motion_seconds': stats['nominal_duration_s']}))


if __name__ == '__main__':
    package('alignment_package', json.loads((HERE/'alignment/candidate.json').read_text()),
            '4-second simulation-validated alignment only', ['align_to_saved_start'])
    package('approach_package', {**source, 'segments': source['segments'][:1]},
            'Saved stage 1 only; stop before hook insertion for physical geometry check', ['side_insert'])
    package('remaining_package', {**source, 'segments': source['segments'][1:]},
            'Saved stages 2-10; HOLD until physical insertion geometry check',
            ['hook_engage', 'test_lift', 'lift', 'transport', 'pre_seat', 'seat_contact', 'unload', 'side_withdraw', 'retreat'])
