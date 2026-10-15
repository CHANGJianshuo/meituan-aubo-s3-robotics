#!/usr/bin/env python3
"""Read-only final audit of this field attempt; never connects to a robot."""
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parent
WORKSPACE = ROOT.parents[2]
sys.path.insert(0, str(WORKSPACE/'scripts/task1_real_scene'))
from fit_kinematics import origins, forward


def read(path):
    return json.loads(path.read_text())


def pinned(path):
    return dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def main():
    packages = read(ROOT/'commissioning/commissioning.json')['packages']
    assert len(packages) == 1 and packages[0]['source_stages'] == [1,30]
    session = read(ROOT/'field_session/status.json')
    assert session['status'] == 'all_packages_completed_pending_placement_verification'
    assert [p['package'] for p in session['completed_packages']] == [packages[0]['directory']]
    bridge_path = ROOT/'field_session'/(packages[0]['directory']+'.events.jsonl')
    bridge_rows = [json.loads(line) for line in bridge_path.read_text().splitlines()]
    assert sum(row['event']=='player_returned' for row in bridge_rows) == 1
    assert not any(row['event'] in ('bridge_error','owned_stop_requested','stop_unverified') for row in bridge_rows)
    remote_audit_path = ROOT/'final_remote_audit.json'
    remote = read(remote_audit_path)
    assert remote['total_motion_intents'] == 30 and not remote['matching_execution_processes']
    assert remote['execution_log_counts'] == {p['directory']:1 for p in packages}
    assert list(remote['ledgers']) == [packages[0]['plan_id']]
    ledger = remote['ledgers'][packages[0]['plan_id']]['content']
    assert ledger['plan_id'] == packages[0]['plan_id'] and ledger['selection'] == 'full'
    reports, rtde, all_rpc, timeline = [], [], [], []
    for package in packages:
        paths = list((ROOT/'nuc_logs'/package['directory']).glob('*.jsonl'))
        assert len(paths) == 1, 'Exactly one NUC attempt log required per package'
        remote_logs = remote['package_logs'][package['directory']]
        assert any(x['sha256'] == pinned(paths[0])['sha256'] and x['event_counts'].get('move_intent',0)>0 for x in remote_logs)
        rows = [json.loads(s) for s in paths[0].read_text().splitlines()]
        count = package['source_stages'][1]-package['source_stages'][0]+1
        intents = [r for r in rows if r['event'] == 'move_intent']
        responses = [r for r in rows if r['event'] == 'move_response']
        endings = [r for r in rows if r['event'] == 'stage_complete']
        completed = [r for r in rows if r['event'] == 'completed']
        reserved = [r for r in rows if r['event'] == 'attempt_reserved']
        assert len(reserved) == len(completed) == 1
        assert reserved[0]['plan_id'] == completed[0]['plan_id'] == package['plan_id']
        for events in [intents, responses, endings]:
            assert [r['stage'] for r in events] == list(range(1,count+1))
        assert all(r['result'] == 0 for r in responses)
        for intent, response, ending in zip(intents, responses, endings):
            assert intent['utc'] <= response['utc'] <= ending['utc']
            timeline.extend([intent['utc'], ending['utc']])
        expected = package['end_q']
        endpoint_error = max(abs(a-b) for a,b in zip(expected,endings[-1]['final_q']))
        assert endpoint_error <= math.radians(.05)
        rpc = [r for r in rows if r['event'] == 'rpc_motion']
        assert rpc and rpc[-1]['queue'] == 0 and rpc[-1]['exec_id'] == -1
        assert max(map(abs,rpc[-1]['qd'])) <= math.radians(.02)
        assert not any(r['event'] in ('failed','stop_response','stop_unverified','stationary_rejected','stationary_timeout') for r in rows)
        assert all(not r['collision'] and r['safety_mode'] == 'Normal' for r in rpc)
        samples = [r for r in rows if r['event'] == 'rtde']
        assert samples
        tracking = max(abs(a-b) for r in samples for a,b in zip(r['q'],r['target_q']))
        assert tracking <= math.radians(.5)
        rtde.extend(samples); all_rpc.extend(rpc)
        reports.append(dict(package=package['directory'], plan_id=package['plan_id'],
            source_stages=package['source_stages'], completed_segments=count, receipt=pinned(paths[0]),
            endpoint_max_error_deg=math.degrees(endpoint_error),
            peak_sampled_joint_tracking_error_deg=math.degrees(tracking),
            started_utc=intents[0]['utc'], completed_utc=completed[0]['utc'], passed=True))
    assert timeline == sorted(timeline)
    assert sum(r['completed_segments'] for r in reports) == 30

    review_path = ROOT/'final_scene_review.json'
    review = read(review_path)
    assert review['passed'] and review['placed_upright'] == ['A','B','C']
    assert review['green_max_handle_shift_px'] <= 2 and review['tool_withdrawn']
    placement_path = ROOT/review['placement_check']
    placement = read(placement_path)
    assert placement['passed'] and [x['object'] for x in placement['objects']] == ['A','B','C']
    record_path = ROOT/review['capture_record']
    record = read(record_path)
    assert placement['capture_record_sha256'] == pinned(record_path)['sha256']
    for name, digest in record['sha256'].items():
        assert pinned(Path(record['capture_directory'])/name)['sha256'] == digest
    for sample in record['telemetry']:
        robot = sample['robot']; d = robot['data']
        assert robot['fresh'] and d['stop_confirmed'] and not d['motion_active']
        assert d['queue'] == 0 and d['exec_id'] == -1 and max(map(abs,d['qd'])) <= math.radians(.02)
        assert max(abs(a-b) for a,b in zip(d['q'],packages[-1]['end_q'])) <= math.radians(.05)

    cfg = read(ROOT/'inputs_01/scenario.json')
    model = origins(ET.parse(cfg['arm_urdf']).getroot())
    q = np.array([r['q'] for r in rtde])
    assert np.isfinite(q).all() and q.shape[1] == 6
    mount = (Rotation.from_euler('z',math.pi/2+math.radians(cfg['tool_mount_yaw_deg'])).as_matrix()
             @ Rotation.from_euler('y',math.pi/2).as_matrix())
    rotations = forward(model,q)[:,:3,:3] @ mount
    reference = Rotation.from_quat(cfg['fixed_tcp_orientation_xyzw']).as_matrix()
    errors = Rotation.from_matrix(reference.T @ rotations).magnitude()
    result = dict(status='PHYSICAL_FULLFLOW_SMOOTH_2X_PASSED',
        created_utc=datetime.now(timezone.utc).isoformat(), order='ABC',
        assignments={'A':'P1','B':'P2','C':'P3'}, completed_motion_segments=30,
        motion_replays=0, packages=reports, rtde_samples=len(rtde), rpc_samples=len(all_rpc),
        physical_tool_orientation=dict(method='Matched controller-model FK of recorded actual joint feedback',
            max_deviation_from_fixed_reference_deg=float(np.degrees(errors.max())),
            absolute_external_orientation_measurement=False, model=pinned(Path(cfg['arm_urdf'])),
            fk_implementation=pinned(WORKSPACE/'scripts/task1_real_scene/fit_kinematics.py')),
        final_capture=pinned(record_path), placement_check=pinned(placement_path),
        remote_log_and_process_audit=pinned(remote_audit_path),
        final_visual_review=pinned(review_path), final_stationary=True,
        final_stable_interval_s=placement['stable_interval_s'],
        reported_controller_collisions=0, interrupted_motion_segments=0,
        fullflow_single_request=True, competition_score_claimed=False,
        nominal_motion_s=read(ROOT/'retiming_02/retiming_validation.json')['planned_motion_s'],
        nominal_motion_ratio=read(ROOT/'retiming_02/retiming_validation.json')['actual_motion_ratio'],
        wall_seconds_first_move_to_completed=(datetime.fromisoformat(reports[-1]['completed_utc'])-datetime.fromisoformat(reports[0]['started_utc'])).total_seconds(),
        original_uniform_tracking_guard_deg=.5,
        peak_sampled_joint_tracking_error_deg=max(r['peak_sampled_joint_tracking_error_deg'] for r in reports),
        recorded_target_peak_velocity_deg_s=[max(abs(math.degrees(r['target_qd'][j])) for r in rtde) for j in range(6)],
        recorded_target_peak_acceleration_deg_s2=[max(abs(math.degrees(r['target_qdd'][j])) for r in rtde) for j in range(6)],
        controller_target_derivative_feedback_usable=any(any(x != 0 for x in r['target_qd']+r['target_qdd']) for r in rtde),
        derivative_feedback_note='The SDK target velocity and acceleration channels are zero throughout this moving run. These values are not evidence of zero physical velocity/acceleration; smoothness bounds refer to the exported polynomial. Position tracking is checked independently.',
        field_session_status=pinned(ROOT/'field_session/status.json'),
        bridge_receipt=pinned(bridge_path), player_returned_normally=True,
        connection_teardown_exceptions=[],
        smooth_polynomial_bounds=pinned(ROOT/'commissioning/exported_polynomial_bounds.json'),
        accepted_simulation_replay=pinned(ROOT/'saved_trajectory/validation.json'),
        validation_program=pinned(Path(__file__)))
    output = ROOT/'physical_validation.json'
    with output.open('x') as f:
        json.dump(result,f,indent=2);f.write('\n')
    print(json.dumps({k:v for k,v in result.items() if k != 'packages'},indent=2))


if __name__ == '__main__':
    main()
