#!/usr/bin/env python3
"""Validate published numerical examples offline. No SDK, ROS or network access."""
import hashlib
import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    spec = importlib.util.spec_from_file_location('public_trajectory',
        ROOT/'hardware/players/tracking_monitor/trajectory.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    folder = ROOT/'examples/field'
    manifest = json.loads((folder/'manifest.json').read_text())
    results = []
    for entry in manifest['files']:
        raw = (folder/entry['file']).read_bytes()
        if hashlib.sha256(raw).hexdigest() != entry['public_sha256']:
            raise ValueError('Public example checksum changed: '+entry['file'])
        data = json.loads(raw)
        if data['accepted_for_robot_motion'] is not False or data['execution_status'] != 'OFFLINE_REFERENCE_ONLY':
            raise ValueError('Example must retain offline-only scope')
        stats = module.validate(data)
        if stats['segments'] != entry['stages'] or abs(stats['nominal_duration_s']-entry['nominal_motion_s']) > 1e-6:
            raise ValueError('Example stage count or timing changed')
        for seg in data['segments']:
            for p in (seg['points'][0], seg['points'][-1]):
                if max(map(abs, p['velocities']+p['accelerations'])) > 1e-9:
                    raise ValueError('Stage endpoint is not at rest')
        results.append(dict(file=entry['file'], **stats))
    print(json.dumps(dict(offline_only=True, passed=True, trajectories=results), indent=2))


if __name__ == '__main__':
    main()
