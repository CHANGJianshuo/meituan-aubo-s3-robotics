#!/usr/bin/env python3
"""Run the preserved task-one RGB-D registration on a new frozen capture."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path


def register(implementation, capture_record, output):
    record = json.loads(capture_record.read_text())
    output.mkdir(parents=True, exist_ok=False)
    (output/'current_placement_capture_request.json').write_text(json.dumps(
        {'capture_directory':record['capture_directory']}, indent=2)+'\n')
    spec = importlib.util.spec_from_file_location('task1_registration', implementation)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    module.OUT = output.resolve()
    module.main()
    (output/'reused_sources.json').write_text(json.dumps(
        {str(implementation.resolve()):hashlib.sha256(implementation.read_bytes()).hexdigest()}, indent=2)+'\n')


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['implementation','capture-record','output']:
        p.add_argument('--'+name,type=Path,required=True)
    register(**vars(p.parse_args()))
