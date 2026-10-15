"""Reject empty, wrong-color, unstable and altered field placement evidence."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts/task2_real_scene'))
spec = importlib.util.spec_from_file_location('verify_placement', ROOT / 'scripts/task2_real_scene/verify_placement.py')
verifier = importlib.util.module_from_spec(spec)
spec.loader.exec_module(verifier)


def fixture(tmp_path, fault=None):
    capture = tmp_path / 'capture'
    capture.mkdir()
    frames = np.zeros((30,400,400,3), dtype=np.uint8)
    for index, image in enumerate(frames):
        shift = 5 if fault == 'unstable' and index == 29 else 0
        image[197:217,190+shift:210+shift] = [0,255,255] if fault == 'wrong_color' else [255,0,0]
    np.savez_compressed(capture/'frames.npz', color_rgb=frames)
    meta = dict(camera=dict(color_intrinsics=dict(fx=800,fy=800,ppx=200,ppy=200)),
                frames=[dict(host_received_ns=i*200_000_000) for i in range(30)])
    (capture/'capture.json').write_text(json.dumps(meta))
    record = dict(capture_directory=str(capture), sha256={
        n:hashlib.sha256((capture/n).read_bytes()).hexdigest() for n in ['frames.npz','capture.json']})
    record_path=tmp_path/'record.json';record_path.write_text(json.dumps(record))
    points=tmp_path/'points.json';points.write_text(json.dumps(dict(order='ABC',
        T_base_color_optical_candidate=[[1,0,0,0],[0,-1,0,0],[0,0,-1,1],[0,0,0,1]],
        targets=dict(P1=dict(base_xyz_m=[0,0,0],center_pixel=[200,200])))))
    if fault == 'altered':
        (capture/'capture.json').write_text('{}')
    return record_path,points


def test_stable_correct_color_at_target_passes(tmp_path):
    record,points=fixture(tmp_path)
    assert verifier.verify(record,points,'A',tmp_path/'result.json')['passed']


def test_motion_during_dwell_rejects_placement(tmp_path):
    record,points=fixture(tmp_path,'unstable')
    assert not verifier.verify(record,points,'A',tmp_path/'result.json')['passed']


@pytest.mark.parametrize('fault,placed',[('wrong_color','A'),('altered','A'),(None,'')])
def test_missing_or_changed_evidence_cannot_pass(tmp_path,fault,placed):
    record,points=fixture(tmp_path,fault)
    with pytest.raises(ValueError):
        verifier.verify(record,points,placed,tmp_path/'result.json')
