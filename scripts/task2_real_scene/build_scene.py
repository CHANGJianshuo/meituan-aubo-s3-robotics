#!/usr/bin/env python3
"""Build fixed-orientation contact simulation using task-one model and current points."""
import argparse
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

TASK1 = Path(__file__).resolve().parents[1]/'task1_real_scene'
sys.path.insert(0, str(TASK1))
# Import by path so this task-two entrypoint cannot import itself as build_scene.
spec = importlib.util.spec_from_file_location('task1_scene', TASK1/'build_scene.py')
task1 = importlib.util.module_from_spec(spec); spec.loader.exec_module(task1)
from fit_kinematics import JOINTS, origins, forward

def source_separations(positions, yaws):
    """Separating-axis gaps for the full 70 x 80 mm source footprints."""
    corners = {}; axes = {}; gaps = {}
    for key, xyz in positions.items():
        if len(xyz) != 3 or not all(math.isfinite(x) for x in xyz) or not math.isfinite(yaws[key]):
            raise ValueError('Invalid source pose: '+key)
        c,s = math.cos(yaws[key]), math.sin(yaws[key])
        R = np.array([[c,-s],[s,c]])
        axes[key] = R.T
        corners[key] = np.array([[x,y] for x in [-.035,.035] for y in [-.04,.04]])@R.T+xyz[:2]
    keys = list(positions)
    for i,a in enumerate(keys):
        for b in keys[i+1:]:
            separation = []
            for axis in [*axes[a], *axes[b]]:
                p,q = corners[a]@axis,corners[b]@axis
                separation.append(max(p.min()-q.max(),q.min()-p.max()))
            gap = float(max(separation)); gaps[a+'_'+b] = gap
            if gap < -.001:
                raise ValueError(f'Candidate source footprints overlap by {-gap*1000:.2f} mm: {a}/{b}; recheck measurement')
    return gaps


def build(reference, measured, output, workspace):
    points = json.loads(measured.read_text())
    order = points['order']
    if (points['frame'] != 'base_link' or points['units'] != 'm/rad'
            or len(order) != 3 or len(set(order)) != 3 or any(c not in 'ABCD' for c in order)):
        raise ValueError('Expected a three-object scene in base_link metres/radians')
    if set(points['targets']) != {'P1','P2','P3'} or set(points['objects']) != set('ABCD'):
        raise ValueError('All three slots and all four physical batteries must be represented')
    cfg = json.loads(reference.read_text())
    for key in ['source_anchor','target_adjustment','target_footprint_size_m','replay_commands',
                'retiming_validation','resume_from_prealigned','recognized_orientation']:
        cfg.pop(key, None)
    positions = {key:value['base_xyz_m'] for key,value in points['objects'].items()}
    yaws = {key:value['yaw_rad'] for key,value in points['objects'].items()}
    separations = source_separations(positions, yaws)
    z = points['support_z_m']
    nominal = json.loads((workspace/'src/mtc_simulation/config/sequence.json').read_text())
    a = np.array([nominal['targets'][p][:2] for p in ['P1','P2','P3']])
    b = np.array([points['targets'][p]['base_xyz_m'][:2] for p in ['P1','P2','P3']])
    u, _, vt = np.linalg.svd((a-a.mean(0)).T@(b-b.mean(0)))
    R = vt.T@u.T
    if np.linalg.det(R) < 0:
        raise ValueError('Mirrored slot labels: verify P1/P2/P3 in the image')
    offset = b.mean(0)-R@a.mean(0)
    angle = math.atan2(R[1,0],R[0,0])
    model = origins(ET.parse(cfg['arm_urdf']).getroot())
    flange = forward(model, points['initial_joints'])[0]
    tool_rotation = (flange[:3,:3]
                     @ Rotation.from_euler('z', math.pi/2+math.radians(cfg['tool_mount_yaw_deg'])).as_matrix()
                     @ Rotation.from_euler('y', math.pi/2).as_matrix())
    if tool_rotation[2,0] < math.cos(math.radians(.5)):
        raise ValueError('Current tool is not level enough for a fixed-orientation insertion')
    fixed = Rotation.from_matrix(tool_rotation).as_quat().tolist()
    direction = tool_rotation[:2,2]/np.linalg.norm(tool_rotation[:2,2])
    acquire = {k:direction.tolist() for k in 'ABCD'}
    delivery = {k:direction.tolist() for k in ['left','right']}
    output.mkdir(parents=True, exist_ok=False)
    cfg.update(task='sequence', requested_order=order, table_z=z,
               initial_objects=positions, object_yaws=yaws,
               initial_joints=dict(zip(JOINTS, points['initial_joints'])),
               targets={p:v['base_xyz_m'] for p,v in points['targets'].items()},
               acquire_directions=acquire, delivery_direction=delivery,
               fixed_tcp_orientation_xyzw=fixed, approach_backoff_m={'left':.06,'right':.06},
               source_footprint_separations_m=separations,
               insertion_tcp_height_above_support_m=.0565,
               world_template=str((output/'world.sdf').resolve()),
               physical_coordinates_verified=False, accepted_for_robot_motion=False,
               target_outer_radius_m=.075, lift_stability_s=2.,
               target_yaw=math.pi/2+angle, board_fit_max_error_m=float(np.linalg.norm(a@R.T+offset-b,axis=1).max()),
               view_transform=dict(T_base_color_optical=points['T_base_color_optical_candidate'],
                                   status=points['status'], source=str(measured.resolve())),
               obstacle_positions_source=dict(method='Current image estimates; all four original batteries retained',
                                              source=str(measured.resolve())))
    cfg['assumptions'].update(perception='Current candidate image points; Gazebo truth for simulation feedback',
        joint_acceleration_jerk='Original task-one planning limits; sequence has not been retimed',
        obstacle_geometry='All four batteries, table, pedestal and rail. Each placed battery remains an obstacle.',
        coordinate_transfer='Task-one support plane and XY anchor correction; task-two physical alignment pending')
    cfg['provenance'].update(points['provenance'])
    cfg['provenance'][str(measured.resolve())] = hashlib.sha256(measured.read_bytes()).hexdigest()
    tree = ET.parse(json.loads(reference.read_text())['world_template'])
    world = tree.getroot().find('world')
    table = world.find("model[@name='table']/link")
    for node in list(table):
        if 'T0_' in node.get('name',''):
            table.remove(node)
    old_board = table.find("visual[@name='visual_original_board']")
    if old_board is not None: table.remove(old_board)
    template = ET.parse(workspace/'src/mtc_simulation/worlds/sequence.sdf').getroot().find('world')
    board = copy.deepcopy(template.find("model[@name='table']/link/visual[@name='visual_original_board']"))
    original_pose = np.fromstring(board.findtext('pose'), sep=' ')
    board.find('pose').text = task1.numbers([*(R@original_pose[:2]+offset), z+.00015,0,0,original_pose[5]+angle])
    table.append(board)
    for node in list(world.findall('model')):
        if node.get('name','').startswith(('battery_', 'neighbor_')):
            world.remove(node)
    for key, xyz in positions.items():
        model = copy.deepcopy(template.find(f"model[@name='battery_{key}']"))
        model.find('pose').text = task1.numbers([*xyz,0,0,yaws[key]])
        body = model.find("link/collision[@name='part_0']")
        geometry = body.find('geometry'); geometry.clear()
        ET.SubElement(ET.SubElement(geometry,'box'),'size').text='.07 .08 .05'
        body_pose = body.find('pose')
        if body_pose is None: body_pose = ET.SubElement(body, 'pose')
        body_pose.text='0 0 .025 0 0 0'
        world.append(model)
    X = np.array(points['T_base_color_optical_candidate'])
    optical_from_gazebo = np.array([[0,-1,0],[0,0,-1],[1,0,0]])
    camera = world.find("model[@name='overview']")
    camera.find('pose').text = task1.numbers([*X[:3,3],*Rotation.from_matrix(X[:3,:3]@optical_from_gazebo).as_euler('xyz')])
    ET.indent(tree); tree.write(output/'world.sdf', encoding='utf-8', xml_declaration=True)
    (output/'scenario.json').write_text(json.dumps(cfg, indent=2)+'\n')
    print(json.dumps(dict(scenario=str(output/'scenario.json'), objects=positions,
                         targets=cfg['targets'], board_fit_max_error_m=cfg['board_fit_max_error_m']), indent=2))
    return cfg


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    for name in ['reference','measured','output']:
        p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--workspace',type=Path,default=Path(__file__).resolve().parents[2])
    build(**vars(p.parse_args()))
