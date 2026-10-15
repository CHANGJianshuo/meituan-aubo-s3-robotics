#!/usr/bin/env python3
"""Regenerate the offline calibration evidence (no ROS, hardware or network).

Runs the unit tests, the documented CLI examples, generated negative inputs and file
format checks.  Every record keeps the exact command, return code, expectation and the
parsed JSON output, so the evidence can be reproduced instead of copied by hand.
Exit 0 only when every expectation holds.

    python3 -B scripts/calibration/run_offline_validation.py [--out DIR]
"""
import argparse
import csv
import datetime
import json
import math
import platform
import re
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CHECK = 'scripts/calibration/check_calibration.py'
DERIVE = 'scripts/calibration/derive_nominal_mount.py'
AUDIT = 'scripts/calibration/audit_fake_model.py'
SYNTHETIC = 'examples/calibration/synthetic_case.json'
UNKNOWN = 'examples/calibration/unknown_extrinsic_case.json'
DESIGN = 'examples/calibration/nominal_mount_design.json'
INVENTORY = 'config/calibration/site_inventory.json'


def run(argv):
    process = subprocess.run([sys.executable, '-B', *argv], cwd=ROOT, capture_output=True,
                             text=True, encoding='utf-8', errors='replace')
    try:
        payload = json.loads(process.stdout)
    except ValueError:
        payload = None
    return process, payload


def close(actual, expected, tol=1e-9):
    return isinstance(actual, list) and len(actual) == len(expected) and \
        all(abs(a - e) <= tol for a, e in zip(actual, expected))


def same_rotation(actual, expected, tol=1e-9):  # q and -q are the same rotation
    return close(actual, expected, tol) or close(actual, [-v for v in expected], tol)


def check_case(name, argv, rc, fields, extra=None, note=''):
    process, payload = run(argv)
    problems = []
    if process.returncode != rc:
        problems.append(f'exit {process.returncode} != {rc}')
    if not isinstance(payload, dict):
        problems.append('stdout is not a JSON object')
        payload = {}
    for key, value in fields.items():
        if payload.get(key) != value:
            problems.append(f'{key}={payload.get(key)!r} != {value!r}')
    if payload.get('motion_authorized') is not False:
        problems.append('motion_authorized is not false')
    if extra:
        problems += extra(payload)
    return {'name': name, 'command': 'python -B ' + ' '.join(argv), 'exit_code': process.returncode,
            'expected_exit_code': rc, 'expected_fields': fields, 'note': note,
            'passed': not problems, 'problems': problems, 'result': payload,
            'stderr_tail': process.stderr[-500:]}


def synthetic_numbers(p):
    s = math.sqrt(0.5)
    out = [] if close(p.get('position'), [1, 1.1, 0.05]) else ['position != [1, 1.1, 0.05]']
    return out + ([] if same_rotation(p.get('orientation_xyzw'), [-s, 0, 0, s]) else ['orientation != Rx(-90 deg)'])


def mount_numbers(p):
    d = p.get('T_F_depth_optical', {})
    out = [] if close(d.get('translation'), [0.0825, 0, 0.0358]) else ['T_F_depth_optical translation changed']
    out += [] if same_rotation(p.get('T_F_camera_link_nominal', {}).get('rotation_xyzw'), [0.5, -0.5, 0.5, 0.5]) \
        else ['T_F_camera_link rotation changed']
    return out


def negative_cases(temp):
    synthetic = json.loads((ROOT / SYNTHETIC).read_text(encoding='utf-8'))
    design = json.loads((ROOT / DESIGN).read_text(encoding='utf-8'))
    inputs = {}
    overflow = json.loads(json.dumps(synthetic))
    overflow['transforms'][0]['translation'][0] = 10**400
    inputs['overflow_snapshot.json'] = (json.dumps(overflow), 'synthetic case, first translation = 10**400')
    overflow_design = dict(design, front_glass_center_in_F=[10**400, 0, 0.04])
    inputs['overflow_mount.json'] = (json.dumps(overflow_design), 'design, glass x = 10**400')
    inputs['deep_nesting.json'] = ('[' * 200000 + ']' * 200000, '200000 nested JSON arrays')
    mm_design = dict(design, front_glass_center_in_F=[100, 0, 40])
    inputs['mm_typed_as_m_mount.json'] = (json.dumps(mm_design), 'design, [100, 0, 40] under length_unit m')
    for filename, (text, _) in inputs.items():
        (temp / filename).write_text(text, encoding='utf-8')
    path = lambda f: str(temp / f)  # noqa: E731
    records = [
        check_case('overflow_snapshot', [CHECK, path('overflow_snapshot.json')], 2,
                   {'status': 'REJECTED', 'code': 'INVALID_VECTOR'}, note=inputs['overflow_snapshot.json'][1]),
        check_case('overflow_mount', [DERIVE, path('overflow_mount.json')], 2,
                   {'status': 'REJECTED', 'code': 'INVALID_VECTOR'}, note=inputs['overflow_mount.json'][1]),
        check_case('deep_nesting_snapshot', [CHECK, path('deep_nesting.json')], 2,
                   {'status': 'REJECTED', 'code': 'INVALID_INPUT'}, note=inputs['deep_nesting.json'][1]),
        check_case('deep_nesting_mount', [DERIVE, path('deep_nesting.json')], 2,
                   {'status': 'REJECTED', 'code': 'INVALID_INPUT'}, note=inputs['deep_nesting.json'][1]),
        check_case('mm_typed_as_m_mount', [DERIVE, path('mm_typed_as_m_mount.json')], 2,
                   {'status': 'REJECTED', 'code': 'IMPLAUSIBLE_DESIGN'}, note=inputs['mm_typed_as_m_mount.json'][1]),
    ]
    # Inputs are regenerated on every run; record them as <tmp>/... rather than this machine's temp path.
    return json.loads(json.dumps(records).replace(json.dumps(str(temp))[1:-1], '<tmp>'))


def format_checks():
    problems = []
    for path in [*ROOT.glob('examples/calibration/*.json'), ROOT / INVENTORY,
                 ROOT / 'docs/calibration/OFFICIAL_SOURCE_RECEIPT.json']:
        try:
            json.loads(path.read_text(encoding='utf-8-sig'))
        except ValueError as error:
            problems.append(f'{path.relative_to(ROOT)}: {error}')
    csv_widths = {}
    for path in sorted(ROOT.glob('docs/calibration/*.csv')):
        widths = {len(r) for r in csv.reader(path.read_text(encoding='utf-8').splitlines())}
        csv_widths[path.name] = sorted(widths)
        if len(widths) != 1:
            problems.append(f'{path.name}: inconsistent column counts {sorted(widths)}')
    links = 0
    for md in [ROOT / 'TF_AND_CALIBRATION_HANDOFF.md', *sorted(ROOT.glob('docs/calibration/*.md'))]:
        for target in re.findall(r'\]\(([^)#\s]+)(?:#[^)]*)?\)', md.read_text(encoding='utf-8')):
            if re.match(r'^[a-z]+://', target):
                continue
            links += 1
            if not (md.parent / target).exists():
                problems.append(f'{md.relative_to(ROOT)}: missing local link {target}')
    inventory = json.loads((ROOT / INVENTORY).read_text(encoding='utf-8'))
    for edge in inventory['transforms']:
        if edge['status'] == 'UNKNOWN' and (edge.get('translation') is not None or edge.get('rotation_xyzw') is not None):
            problems.append(f"site_inventory {edge['parent']}->{edge['child']}: UNKNOWN with numbers")
    if any(v is not None for k, v in inventory['thresholds'].items() if k != 'status'):
        problems.append('site_inventory thresholds must stay null until the team decides them')
    return {'passed': not problems, 'problems': problems, 'csv_column_counts': csv_widths, 'local_links_checked': links}


def git_state():
    try:
        head = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=ROOT, capture_output=True, text=True, check=True).stdout.strip()
        dirty = subprocess.run(['git', 'status', '--porcelain=v1', '--', 'TF_AND_CALIBRATION_HANDOFF.md', 'config/calibration',
                                'docs/calibration', 'examples/calibration', 'scripts/calibration', 'tests/calibration'],
                               cwd=ROOT, capture_output=True, text=True, check=True).stdout.splitlines()
        return {'head': head, 'uncommitted_calibration_paths': dirty}
    except (OSError, subprocess.CalledProcessError):
        return {'head': 'NOT AVAILABLE', 'uncommitted_calibration_paths': []}


def write_json(path, payload):
    path.write_text(json.dumps(payload, ensure_ascii=True, indent=2, allow_nan=False) + '\n', encoding='utf-8', newline='\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--out', type=Path, default=ROOT / 'docs/calibration/validation')
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    tests = subprocess.run([sys.executable, '-B', '-m', 'unittest', 'discover', '-s', 'tests/calibration', '-v'],
                           cwd=ROOT, capture_output=True, text=True, encoding='utf-8', errors='replace')
    (args.out / 'unittest.txt').write_text(tests.stderr, encoding='utf-8', newline='\n')
    ran = re.search(r'^Ran (\d+) tests?', tests.stderr, re.M)
    skipped = re.search(r'skipped=(\d+)', tests.stderr)
    unittest_record = {'command': 'python -B -m unittest discover -s tests/calibration -v', 'exit_code': tests.returncode,
                       'tests_ran': int(ran.group(1)) if ran else None, 'skipped': int(skipped.group(1)) if skipped else 0,
                       'passed': tests.returncode == 0 and not skipped}

    documented = [
        check_case('synthetic', [CHECK, SYNTHETIC], 0,
                   {'status': 'DIAGNOSTIC_OK', 'simulation_only': True, 'calibration_status': 'SYNTHETIC_ONLY',
                    'stamp_ns': 10_000_000_000}, synthetic_numbers, 'pure math fixture, not hardware data'),
        check_case('unknown_extrinsic', [CHECK, UNKNOWN], 2, {'status': 'REJECTED', 'code': 'UNKNOWN_CALIBRATION'},
                   note='UNKNOWN mount edge must not become identity'),
        check_case('nominal_mount', [DERIVE, DESIGN], 0, {'status': 'ESTIMATED', 'deployment_ready': False},
                   mount_numbers, 'design estimate only'),
        check_case('design_not_real_case', [CHECK, DESIGN], 2, {'status': 'REJECTED', 'code': 'INVALID_CONFIG'},
                   note='schema mismatch only; status isolation is covered by unit tests'),
        check_case('site_inventory_not_a_case', [CHECK, INVENTORY], 2, {'status': 'REJECTED', 'code': 'INVALID_POSE'},
                   note='inventory has no observation and cannot produce a target'),
        check_case('fake_model_audit', [AUDIT], 0,
                   {'all_passed': True, 'evidence_kind': 'FAKE_RUNTIME_CAPTURE',
                    'real_calibration_verified': False},
                   note='archived mock hardware model and TF logs; no live ROS or physical measurements'),
    ]
    for record in documented[:4] + documented[5:]:
        write_json(args.out / f"{record['name']}.json", record['result'])
    with tempfile.TemporaryDirectory() as temp:
        negatives = negative_cases(Path(temp))
    write_json(args.out / 'generated_negative_inputs.json', negatives)
    formats = format_checks()

    all_passed = unittest_record['passed'] and formats['passed'] and all(r['passed'] for r in documented + negatives)
    write_json(args.out / 'results.json', {
        'generated_at': datetime.datetime.now().astimezone().isoformat(timespec='seconds'),
        'generator': 'python -B scripts/calibration/run_offline_validation.py',
        'environment': {'python': platform.python_version(), 'platform': sys.platform,
                        'ROS': 'NOT RUN', 'hardware': 'NOT RUN', 'network': 'NOT USED'},
        'git': git_state(), 'unittest': unittest_record,
        'documented_cases': documented, 'generated_negative_cases': [
            {k: r[k] for k in ('name', 'command', 'exit_code', 'expected_exit_code', 'passed', 'problems', 'note')}
            for r in negatives],
        'format_checks': formats, 'all_passed': all_passed,
        'scope': 'offline mechanism evidence only; not hand-eye accuracy, robot motion or competition tolerance'})
    print(json.dumps({'all_passed': all_passed, 'tests_ran': unittest_record['tests_ran'],
                      'failed': [r['name'] for r in documented + negatives if not r['passed']] +
                                ([] if unittest_record['passed'] else ['unittest']) +
                                ([] if formats['passed'] else formats['problems'])}, ensure_ascii=True))
    return 0 if all_passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
