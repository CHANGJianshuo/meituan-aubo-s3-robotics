"""Verify and archive this completed hardware run. No hardware I/O."""
from pathlib import Path
from datetime import datetime, timezone
import csv
import hashlib
import json
import sys
import xml.etree.ElementTree as ET

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
ATT = HERE.parent
REV = ATT.parent
BASE = REV.parent
ROOT = BASE.parent.parent
PACKAGE = ATT/'half_time_package'


def read(path):
    return json.loads(path.read_text())


def save(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False)+'\n')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


log = next((HERE/'hardware_logs').glob('*.jsonl'))
events = [json.loads(x) for x in log.read_text().splitlines()]
endpoints = [x for x in events if x['event'] == 'stage_complete']
moves = [x for x in events if x['event'] == 'move_intent']
completed = [x for x in events if x['event'] == 'completed']
assert len(endpoints) == len(moves) == 10 and len(completed) == 1
assert not any(x['event'] == 'failed' for x in events)
bundle = read(PACKAGE/'task_one_trajectory.json')
source = read(HERE/'saved_trajectory/task_one_trajectory.json')
manifest = read(PACKAGE/'manifest.json')
assert completed[0]['plan_id'] == bundle['plan_id'] == manifest['plan_id']
assert bundle['segments'] == source['segments']
assert manifest['source_saved_trajectory_sha256'] == digest(HERE/'saved_trajectory/task_one_trajectory.json')
for name, expected in {**manifest['sha256'], **manifest['adapter_sha256']}.items():
    assert digest(PACKAGE/name) == expected
for name in ['replay_02', 'source_minus_y2mm_replay']:
    r = read(HERE/name/'run.json')
    assert r['success'] and r['shutdown_clean'] and read(HERE/name/'acceptance.json')['passed']
    assert (HERE/name/'commands.jsonl').read_bytes() == (HERE/'candidate_commands.jsonl').read_bytes()
state = read(HERE/'state_after_completion.json')
assert state['read_only'] and state['max_joint_speed_deg_s'] < .02
assert all(s['robot_mode'] == 'Running' and s['safety_mode'] == 'Normal'
           and s['exec_id'] == -1 and s['queue'] == 0 for s in state['samples'])
final = state['samples'][-1]
endpoint_error = float(np.rad2deg(np.abs(np.array(final['q'])-
                                     bundle['segments'][-1]['points'][-1]['positions']).max()))
assert endpoint_error < .05
tracking = max(float(np.rad2deg(np.abs(np.array(x['q'])-x['target_q']).max()))
               for x in events if x['event'] == 'rtde')
assert tracking < .5
elapsed = (datetime.fromisoformat(completed[0]['utc'])-
           datetime.fromisoformat(moves[0]['utc'])).total_seconds()
previous = read(ATT/'one_shot_01/physical_result.json')['first_motion_to_completed_s']
capture = Path(read(HERE/'final_capture_request.json')['directory'])
frames = np.load(capture/'frames.npz')['color_rgb']
meta = read(capture/'capture.json')
centers, areas = [], []
for im in frames:
    hsv = cv2.cvtColor(im, cv2.COLOR_RGB2HSV)
    mask = (((hsv[:, :, 0] < 12) | (hsv[:, :, 0] > 170)) &
            (hsv[:, :, 1] > 100) & (hsv[:, :, 2] > 65)).astype(np.uint8)
    mask[:450] = 0
    mask[580:] = 0
    mask[:, :450] = 0
    mask[:, 610:] = 0
    _, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)
    i = 1+np.argmax(stats[1:, 4])
    centers.append(centroids[i])
    areas.append(int(stats[i, 4]))
centers = np.array(centers)
deviation = float(np.linalg.norm(centers-centers.mean(0), axis=1).max())
assert min(areas) > 3000 and deviation < .5
cv2.imwrite(str(HERE/'physical_final.png'), cv2.cvtColor(frames[-1], cv2.COLOR_RGB2BGR))
visual = dict(capture=str(capture), frames=len(frames),
    stable_interval_s=(meta['frames'][-1]['host_received_ns']-meta['frames'][0]['host_received_ns'])/1e9,
    maximum_red_centroid_deviation_px=deviation,
    observation='Battery upright in T0 and tool withdrawn, visually reviewed; no person touched the battery in this capture.',
    measurement='Image stability evidence, not physical millimetre placement metrology')
save(HERE/'final_visual_observation.json', visual)

sys.path.insert(0, str(ROOT/'scripts/task1_real_scene'))
from fit_kinematics import origins, forward
model = origins(ET.parse(BASE/'inputs/controller_matched_arm.urdf').getroot())
labels = ['对中 K03', '水平插入', '试提', '抬升', 'T0 上方', '预落位', '落座', '卸载', '水平退出', '抬高退出']
measured = []
for event, segment, label in zip(endpoints, bundle['segments'], labels):
    q = event['final_q']
    f = forward(model, [q])[0]
    error = float(np.rad2deg(np.abs(np.array(q)-segment['points'][-1]['positions']).max()))
    assert error < .05
    measured.append(dict(stage=event['stage'], primitive=segment['primitive'], name_zh=label,
        utc=event['utc'], q_rad=q, q_deg=np.rad2deg(q).tolist(),
        flange_base_xyz_mm=(f[:3, 3]*1000).tolist(),
        nominal_tool_tcp_base_xyz_mm=((f[:3, 3]+f[:3, :3]@np.array([0., .045, .1105]))*1000).tolist(),
        endpoint_max_joint_error_deg=error))
save(HERE/'measured_endpoints.json', dict(frame='physical base_link', log=str(log),
    log_sha256=digest(log), points=measured, nominal_tool_note='CAD-derived; controller TCP offset is zero'))
with (HERE/'measured_endpoints.csv').open('w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['stage', 'primitive', 'utc']+[f'J{i}_deg' for i in range(1, 7)]+['flange_x_mm', 'flange_y_mm', 'flange_z_mm'])
    for p in measured:
        w.writerow([p['stage'], p['primitive'], p['utc'], *p['q_deg'], *p['flange_base_xyz_mm']])
smooth = read(HERE/'retiming_validation.json')
result = dict(status='PHYSICAL_SMOOTH_FAST_TASK_COMPLETED', task_complete=True,
    plan_id=bundle['plan_id'], start_utc=moves[0]['utc'], completed_utc=completed[0]['utc'],
    segments_completed=10, launch_count=1, manual_adjustments_during_run=0,
    first_motion_to_completed_s=elapsed, previous_first_motion_to_completed_s=previous,
    elapsed_ratio=elapsed/previous, time_reduction_percent=100*(1-elapsed/previous),
    exact_half_target_s=previous/2, difference_from_exact_half_s=elapsed-previous/2,
    exact_half_time_met=False, approximate_half_time_achieved=True,
    nominal_motion_s=smooth['planned_motion_s'], measured_checks_and_settling_s=elapsed-smooth['planned_motion_s'],
    endpoint_max_joint_error_deg=endpoint_error, maximum_recorded_tracking_error_deg=tracking,
    final_robot_mode=final['robot_mode'], final_safety_mode=final['safety_mode'], stationary=True, queue_empty=True,
    simulated_and_executed_segments_identical=True, source_offset_simulation_passed=True,
    smoothing='C4 waypoint spline, quintic clock-rate ramps, zero endpoint velocity and acceleration; original K03 and all stage endpoints retained',
    maximum_nominal_tool_path_deviation_mm=max(p['spline_path_max_tool_deviation_mm'] for p in smooth['part_validation']),
    command_peak_velocity_deg_s=max(smooth['statistics']['peak_velocity_deg_s']),
    command_peak_acceleration_deg_s2=max(smooth['statistics']['peak_acceleration_deg_s2']),
    physical_absolute_coordinate_accuracy_verified=False, final_visual_observation=visual,
    trajectory=str(PACKAGE/'task_one_trajectory.json'), trajectory_sha256=digest(PACKAGE/'task_one_trajectory.json'),
    saved_simulation_trajectory=str(HERE/'saved_trajectory/task_one_trajectory.json'),
    saved_simulation_trajectory_sha256=digest(HERE/'saved_trajectory/task_one_trajectory.json'),
    hardware_log=str(log), hardware_log_sha256=digest(log),
    cleanup_issue='RTDE unsubscribe timeout after completed; separate read-only connection confirmed stationary; no motion retried',
    timing_scope='First move_intent to completed, including all stage checks. Initial login and final RTDE shutdown are excluded consistently for both runs.')
save(HERE/'physical_result.json', result)
save(HERE/'progress.json', dict(status=result['status'], task_complete=True, motion_started=True,
    completed_utc=result['completed_utc'], result=str(HERE/'physical_result.json')))
md = f'''# 任务一平滑提速：完整实机执行成功

红电池已搬至 T0，舌片卸载、退出并抬高。一次启动完成全部十段，中途没有人工微调。结束后独立读取确认 Running / Normal、关节静止、队列为空。

| 指标 | 上一轮 | 本轮 |
|---|---:|---:|
| 完整执行时间，含段间检查 | {previous:.3f} 秒 | {elapsed:.3f} 秒 |
| 轨迹纯运动时间 | 142.392 秒 | {result['nominal_motion_s']:.3f} 秒 |

总耗时减少 **{result['time_reduction_percent']:.2f}%**，为原来的 **{elapsed/previous*100:.2f}%**，接近减半。严格一半目标为 {previous/2:.3f} 秒，本次多 {elapsed-previous/2:.3f} 秒。两次均从首次运动计到最后一段检查完成；初始登录与退出 RTDE 的时间不计。

对原路点进行五次样条平滑并重新分配时间，保留 K03 和各段终点；各段起停速度、加速度为零。名义工具路径相对成功原轨迹最大变化 {result['maximum_nominal_tool_path_deviation_mm']:.4f} mm。新轨迹峰值关节速度 {result['command_peak_velocity_deg_s']:.3f}°/秒，加速度 {result['command_peak_acceleration_deg_s2']:.3f}°/秒²；采样 jerk 未超过设计上限 200°/秒³。

完整原样仿真和源电池沿基座 −Y 偏移 2 mm 的仿真均通过。原播放器的起点、配置、限位、跟踪误差、独占锁及单次执行检查保持不变。实机最大记录跟踪误差 {tracking:.6f}°，最终最大关节误差 {endpoint_error:.6f}°。这些不是物块落位精度。

相机连续 {visual['stable_interval_s']:.2f} 秒、{visual['frames']} 帧显示物块立于 T0、工具已分离，红色区域质心最大变化 {deviation:.3f} 像素。相机视角曾发生变化，本轮复位比较采用背景及底图配准，没有复用旧像素当成新测量。

- [13 个成功关键点，含 J6 微转和 −Y 3 mm 修正](../successful_keypoints/KEYPOINTS.md)
- [本轮实测十段终点](measured_endpoints.json) · [CSV](measured_endpoints.csv)
- [完整轨迹 JSON](saved_trajectory/task_one_trajectory.json) · [轨迹 CSV](saved_trajectory/task_one_trajectory.csv) · [实际执行包](../half_time_package/task_one_trajectory.json)
- [实机录像](physical_observation.mp4) · [正常速度仿真录像](replay_02/overview_realtime.mp4) · [完成照片](physical_final.png)
- [执行结果和哈希](physical_result.json) · [平滑验证](retiming_validation.json) · [执行日志](hardware_logs/{log.name})

原 J6 微转及基座 −Y 3 mm 补偿已包含在 K03，不能再重复叠加。本次复现依赖原 V1 安装、同一起抓位置与方向及同一底图。成功执行不等于重新验证了手眼标定绝对精度。

录像来自约 1 Hz 预览，以 5 fps 保持上一帧记录；不是相机原始全帧率视频。播放器在 completed 后关闭 RTDE 订阅时超时；独立只读连接确认关节静止，没有重发动作。包中 HOLD 等字段保留执行前快照，本报告和日志记录实际结果。

电脑目录：`{HERE}`。

NUC 执行包：`/home/robot/meituan_task1_revision01_20261014T184851Z/half_time_package`。
'''
(HERE/'RESULT.md').write_text(md)
status = dict(status=result['status'], updated_utc=datetime.now(timezone.utc).isoformat(),
    task_complete=True, original_trajectories_preserved=True, physical_execution_validated=True,
    latest_successful_execution=str(HERE/'physical_result.json'), report=str(HERE/'RESULT.md'),
    latest_trajectory=str(PACKAGE/'task_one_trajectory.json'), trajectory_sha256=result['trajectory_sha256'],
    keypoint_record=str(ATT/'successful_keypoints/KEYPOINTS.md'), latest_measured_endpoints=str(HERE/'measured_endpoints.json'),
    source_scene=str(HERE/'scenario.json'), normal_speed_video=str(HERE/'replay_02/overview_realtime.mp4'),
    physical_video=str(HERE/'physical_observation.mp4'), real_elapsed_s=elapsed,
    original_elapsed_s=previous, time_reduction_percent=result['time_reduction_percent'],
    validation_scope='Supervised single full run of this smoothed trajectory at the restored source placement',
    absolute_physical_geometry_verified=False, automatic_replay_from_current_pose_allowed=False)
save(BASE/'trajectory_use_status.json', status)
save(REV/'latest_physical_attempt.json', dict(directory=str(HERE), motion_started=True,
    task_complete=True, result=str(HERE/'physical_result.json')))
for path, relative in [(BASE/'RESULT.md', 'revision_01/physical_attempt_20261014T184851Z/half_time_01/RESULT.md'),
                       (REV/'RESULT.md', 'physical_attempt_20261014T184851Z/half_time_01/RESULT.md'),
                       (ATT/'RESULT.md', 'half_time_01/RESULT.md')]:
    old = path.read_text()
    path.write_text(f'最新：平滑提速版本已一次完成十段实机流程。总耗时 {elapsed:.3f} 秒，原为 {previous:.3f} 秒，减少 {result["time_reduction_percent"]:.2f}%；机械臂已退出并静止。见 [最终执行记录]({relative})。\n\n以下保留之前版本的历史记录。\n\n'+old)
evidence = [HERE/'physical_result.json', HERE/'RESULT.md', HERE/'state_after_completion.json',
            HERE/'measured_endpoints.json', HERE/'physical_observation.mp4',
            HERE/'physical_final.png', HERE/'retiming_validation.json', log,
            PACKAGE/'task_one_trajectory.json', HERE/'saved_trajectory/task_one_trajectory.json']
save(HERE/'evidence_manifest.json', {str(p): digest(p) for p in evidence})
print(json.dumps({k: result[k] for k in ['status', 'first_motion_to_completed_s', 'time_reduction_percent',
    'endpoint_max_joint_error_deg', 'maximum_recorded_tracking_error_deg']}, indent=2))
