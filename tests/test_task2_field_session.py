"""Task-two stage boundaries and inherited bridge supervision contracts."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import threading

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('field_session', ROOT / 'scripts/task2_real_scene/field_session.py')
session = importlib.util.module_from_spec(spec)
spec.loader.exec_module(session)


def fixture(tmp_path):
    package = tmp_path / 'commissioning/01_A_P1_approach'
    package.mkdir(parents=True)
    (package / 'manifest.json').write_text('{}')
    (tmp_path / 'evidence.json').write_text('{"passed":true}')
    entry = dict(directory=package.name, plan_id='task-two-stage-one',
                 manifest_sha256=session.sha(package / 'manifest.json'))
    request = dict(command='execute', package=package.name, created_unix=100., request_id='unique-1',
                   reviewed_evidence={'evidence.json': session.sha(tmp_path / 'evidence.json')})
    return entry, request


def test_reviewed_next_stage_is_accepted(tmp_path):
    entry, request = fixture(tmp_path)
    session.validate_request(request, entry, tmp_path, now=101.)


@pytest.mark.parametrize('fault', ['wrong_stage', 'expired', 'no_evidence', 'changed_evidence', 'changed_manifest'])
def test_rejects_unreviewed_or_changed_execution_request(tmp_path, fault):
    entry, request = fixture(tmp_path)
    if fault == 'wrong_stage':
        request['package'] = '01_A_P1_insert_trial_lift'
    elif fault == 'expired':
        request['created_unix'] = 0.
    elif fault == 'no_evidence':
        request['reviewed_evidence'] = {}
    elif fault == 'changed_evidence':
        (tmp_path / 'evidence.json').write_text('{"passed":false}')
    else:
        (tmp_path / 'commissioning' / entry['directory'] / 'manifest.json').write_text('{"modified":true}')
    with pytest.raises(ValueError):
        session.validate_request(request, entry, tmp_path, now=101.)


@pytest.mark.parametrize('outcome', ['completed', 'missing_receipt', 'stop',
                                   'timeout_complete', 'timeout_incomplete', 'timeout_owned_stop'])
def test_stage_requires_receipts_and_preserves_stop_channel(tmp_path, monkeypatch, outcome):
    entry, _ = fixture(tmp_path)
    bridge = tmp_path / 'commissioning/remote_bridge.py'
    bridge.write_text('# pinned adapter fixture\n')
    deployment = dict(remote_directory='/remote/commissioning', ssh_host='example', transport_options=[],
                      sha256={'remote_bridge.py': session.sha(bridge)})
    folder = tmp_path / 'field_session'
    folder.mkdir()
    if outcome == 'stop':
        (folder / 'stop.request').touch()
    script = '''import json,sys
auth=json.loads(sys.stdin.readline())
assert auth['password']=='sentinel-password'
message=json.loads(sys.stdin.readline())
def emit(**data): print('MTC_HARDWARE '+json.dumps(data),flush=True)
if OUTCOME=='stop':
    assert message['command']=='stop'
    emit(event='owned_stop_requested')
    raise SystemExit(1)
assert message['command']=='ping'
if OUTCOME=='completed':
    emit(event='completed',plan_id='task-two-stage-one')
if OUTCOME.startswith('timeout_'):
    if OUTCOME!='timeout_incomplete': emit(event='completed',plan_id='task-two-stage-one')
    if OUTCOME=='timeout_owned_stop': emit(event='owned_stop_requested')
    print('[Error] RTDE send TIMEOUT',file=sys.stderr,flush=True)
    raise SystemExit(255)
emit(event='player_returned')
'''.replace('OUTCOME', repr(outcome))
    real_popen = subprocess.Popen
    calls = []

    def popen(argv, **kwargs):
        calls.append(argv)
        return real_popen([sys.executable, '-u', '-c', script], **kwargs)

    monkeypatch.setattr(session.subprocess, 'Popen', popen)
    stop = threading.Event()
    credentials = dict(username='user', password='sentinel-password')
    run = lambda: session.run_stage(tmp_path, folder, deployment, entry, credentials, stop, lambda *_a, **_k: None)
    if outcome == 'completed':
        run()
    elif outcome == 'timeout_complete':
        with pytest.raises(session.CompletedTeardownError):
            run()
    else:
        with pytest.raises(RuntimeError) as caught:
            run()
        assert not isinstance(caught.value, session.CompletedTeardownError)
    assert len(calls) == 1  # No retry, including failed/missing receipts.
    assert 'sentinel-password' not in repr(calls)
    assert all('sentinel-password' not in p.read_text() for p in folder.iterdir() if p.is_file())
    if outcome == 'stop':
        assert stop.is_set()


@pytest.mark.parametrize('fault',[None,'missing_segment','skipped_package','wrong_current_pose','invalid_final_q'])
def test_resume_requires_complete_prefix_and_independent_stationary_endpoint(tmp_path,fault):
    entry,_=fixture(tmp_path)
    entry.update(source_stages=[1,1],end_q=[0.]*6,start_q=[0.]*6)
    next_entry=dict(entry,directory='01_A_P1_insert_trial_lift',plan_id='next')
    rows=[dict(event='move_response',stage=1,result=0),
          dict(event='stage_complete',stage=1,final_q=[0.]*6),
          dict(event='completed',plan_id=entry['plan_id'])]
    if fault=='missing_segment': rows.pop(1)
    if fault=='invalid_final_q': rows[1]['final_q']=[0.]
    log=tmp_path/'receipt.jsonl';log.write_text(''.join(json.dumps(x)+'\n' for x in rows))
    q=[.01]*6 if fault=='wrong_current_pose' else [0.]*6
    record=tmp_path/'capture.json';record.write_text(json.dumps(dict(read_only=True,telemetry=[dict(robot=dict(
        fresh=True,data=dict(stop_confirmed=True,motion_active=False,queue=0,exec_id=-1,q=q,qd=[0.]*6)))])))
    def item(p): return dict(path=p.name,sha256=session.sha(p))
    checkpoint=dict(status='VERIFIED_STOPPED_BETWEEN_STAGES',
        completed_packages=[dict(package='wrong' if fault=='skipped_package' else entry['directory'],receipt=item(log))],
        next_package=next_entry['directory'],stationary_capture_record=item(record),
        reviewed_evidence=[item(tmp_path/'evidence.json')])
    path=tmp_path/'checkpoint.json';path.write_text(json.dumps(checkpoint))
    if fault:
        with pytest.raises(ValueError):session.verified_checkpoint(tmp_path,[entry,next_entry],path)
    else:
        checked=session.verified_checkpoint(tmp_path,[entry,next_entry],path)
        assert [x['package'] for x in checked]==[entry['directory']]
        request=dict(command='reconcile_completed',created_unix=100,checkpoint=path.name,sha256=session.sha(path))
        assert session.reconcile_request(request,tmp_path,[entry,next_entry],0,now=101)==checked


@pytest.mark.parametrize('fault', ['expired','changed_hash','skip_package','wrong_command'])
def test_reconciliation_rejects_stale_changed_or_skipped_checkpoint(tmp_path,monkeypatch,fault):
    path=tmp_path/'checkpoint.json';path.write_text('{}')
    request=dict(command='reconcile_completed',created_unix=100,checkpoint=path.name,sha256=session.sha(path))
    if fault=='expired':request['created_unix']=0
    if fault=='changed_hash':path.write_text('{"changed":true}')
    if fault=='wrong_command':request['command']='execute'
    checked=[dict(package='completed')]*(2 if fault=='skip_package' else 1)
    monkeypatch.setattr(session,'verified_checkpoint',lambda *_:checked)
    with pytest.raises(ValueError):session.reconcile_request(request,tmp_path,[],0,now=101)
