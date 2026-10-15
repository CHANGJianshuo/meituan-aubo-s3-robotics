"""Package the accepted half-time trajectory using the unchanged hardware player."""
from pathlib import Path
import hashlib
import importlib.util
import json
import shutil

HERE = Path(__file__).resolve().parent
ATT = HERE.parent
for name in ['replay_02', 'source_minus_y2mm_replay']:
    run = json.loads((HERE/name/'run.json').read_text())
    acceptance = json.loads((HERE/name/'acceptance.json').read_text())
    assert run['success'] and run['shutdown_clean'] and acceptance['passed']
    assert (HERE/name/'commands.jsonl').read_bytes() == (HERE/'candidate_commands.jsonl').read_bytes()

spec = importlib.util.spec_from_file_location('package_builder', ATT/'build_packages.py')
builder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(builder)
builder.state = json.loads((HERE/'state_before_execution.json').read_text())
# Correct provenance to this exact accepted trajectory rather than the earlier
# revision ancestor used by the original stage-by-stage packaging helper.
builder.source_path = HERE/'saved_trajectory/task_one_trajectory.json'
builder.source = json.loads(builder.source_path.read_text())
builder.package('half_time_package', builder.source,
    'One invocation of all ten smoothed stages, preserving K03 and all stage endpoints. '
    'Requested total wall time approximately half of the 175.872-second successful run. '
    'Complete nominal and minus-Y 2 mm source-offset simulations passed. No manual corrections.',
    [s['primitive'] for s in builder.source['segments']])
out = ATT/'half_time_package'
for name in ['retiming_validation.json', 'source_stability.json', 'source_local_registration.json',
             'source_registration_diagnostic.json', 'state_before_execution.json']:
    shutil.copyfile(HERE/name, out/name)
shutil.copyfile(ATT/'successful_keypoints/keypoints.json', out/'successful_keypoints.json')
for source, target in [('replay_02/acceptance.json', 'simulation_acceptance.json'),
                       ('source_minus_y2mm_replay/acceptance.json', 'offset_acceptance.json')]:
    shutil.copyfile(HERE/source, out/target)
(out/'README.md').write_text(
    '# 任务一：平滑提速版本\n\n'
    '一次启动完整执行十段；控制柜检查、0.5° 跟踪误差停止阈值、限位、独占锁和单次执行台账保持不变。'
    '轨迹已完成名义场景及源电池沿基座 −Y 偏移 2 mm 的原样仿真回放。'
    'K03 和各段终点不变，各段起停速度、加速度为零；速度不超过 13°/秒，'
    '加速度不超过 25°/秒²，jerk 不超过 200°/秒³。以上为本次轨迹设计上限。\n\n'
    '执行前快照；实机结果另记于 half_time_01/physical_result.json。\n', encoding='utf-8')
print(json.dumps({'package': str(out), 'trajectory_sha256': hashlib.sha256(
    (out/'task_one_trajectory.json').read_bytes()).hexdigest()}, indent=2))
