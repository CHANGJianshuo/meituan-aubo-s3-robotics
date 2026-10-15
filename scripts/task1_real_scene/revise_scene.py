#!/usr/bin/env python3
"""Build a versioned simulation from reviewed object and support-plane estimates.

Consumes diagnostic evidence; does not certify coordinates for physical motion.
Original scene, point measurements and trajectory remain untouched.
"""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
from build_scene import add_box, numbers
from fit_kinematics import JOINTS


def build(original_inputs, audit, output):
    output.mkdir(parents=True, exist_ok=False)
    cfg = json.loads((original_inputs/'scenario.json').read_text())
    original_cfg = copy.deepcopy(cfg)
    source = json.loads((audit/'current_object_placement.json').read_text())
    target = json.loads((audit/'current_T0_placement.json').read_text())
    view = json.loads((audit/'camera_registration_candidate.json').read_text())
    state = json.loads((audit/'object_placement_current_state.json').read_text())
    anchor = np.array(source['candidate_handle_top_base_mm'])/1000
    origin = np.array(source['candidate_battery_origin_base_mm'])/1000
    T0 = np.array(target['target_base_mm'])/1000
    assert abs(T0[2]-origin[2]) < 1e-9
    X = np.array(view['T_base_current_camera_candidate'])
    capture = Path(source['capture_directory'])
    z = float(origin[2])
    target_yaw = target['target_yaw_rad']
    # User requested the red battery only in this revised simulation.
    object_yaws = {'A': cfg['yaw']}
    positions = {'A': origin.tolist()}
    boxes = [
        dict(id='table', xyz=[.2, 0., z-.025-.0003], size=[.8, 1.4, .05], yaw=0.),
        dict(id='base_pedestal', xyz=[0., 0., z/2-.0003], size=[.18, .18, -z], yaw=0.),
        dict(id='aluminum_rail_envelope', xyz=[.075, 0., -.035], size=[.05, 1.3, .06], yaw=0.),
    ]
    deliver = [-math.sin(target_yaw), math.cos(target_yaw)]
    cfg.update(table_z=z, initial_objects=positions, object_yaws=object_yaws, targets={'T0': T0.tolist()}, target_yaw=target_yaw,
               initial_joints=dict(zip(JOINTS, state['samples'][-1]['q'])), scene_boxes=boxes,
               delivery_direction={'left': deliver, 'right': deliver}, transport_tcp_z=z+.35,
               insertion_tcp_height_above_support_m=.0565,
               world_template=str((output/'world.sdf').resolve()),
               source_anchor=dict(base_xyz_m=anchor.tolist(), semantics='geometric top surface centre of crossbar',
                                  status='ESTIMATED_FROM_CURRENT_IMAGE_AND_APPROXIMATE_GAP'),
               target_adjustment=dict(original_base_xyz_m=original_cfg['target_adjustment']['original_base_xyz_m'],
                                      previous_corrected_base_xyz_m=original_cfg['targets']['T0'],
                                      corrected_base_xyz_m=T0.tolist(),
                                      source='User confirmed source and target supports are essentially the same height; current T0 contour intersected with that plane'),
               surface_model=dict(source_support_z_m=z, target_support_z_m=z, local_height_difference_mm=0.,
                                  interpretation='User-confirmed approximately common support plane; small unevenness unmeasured'),
               physical_coordinates_verified=False, accepted_for_robot_motion=False,
               obstacle_positions_source=dict(non_target_batteries='omitted by explicit user request'),
               view_transform=dict(T_base_color_optical=X.tolist(), status='DIAGNOSTIC_EFFECTIVE_VIEW_REGISTRATION', source=str(audit/'camera_registration_candidate.json')))
    cfg['assumptions']['perception'] = 'Reviewed approximate object pose; simulation feedback uses Gazebo truth; independent physical coordinate validation pending'
    cfg['assumptions']['surface_height'] = 'Common plane inferred from approximate 16 mm tool-to-bar gap and nominal 80 mm battery; not a surveyed table'
    cfg['assumptions']['obstacle_geometry'] = 'Only red battery, table, T0, base pedestal and rail envelope; other batteries removed by user request'
    cfg['acquire_directions'] = {'A': cfg['acquire_directions']['A']}
    cfg['recognized_orientation']['target_long_axis_yaw_deg'] = math.degrees(target_yaw)
    for f in [audit/'current_object_placement.json', audit/'current_T0_placement.json',
              audit/'camera_registration_candidate.json', audit/'onsite_observation.json',
              audit/'object_placement_current_state.json']:
        cfg['provenance'][str(f.resolve())] = hashlib.sha256(f.read_bytes()).hexdigest()

    tree = ET.parse(original_inputs/'measured_world.sdf')
    world = tree.getroot().find('world')
    link = world.find("model[@name='table']/link")
    board = copy.deepcopy(link.find("visual[@name='visual_original_board']"))
    old_board_pose = np.fromstring(board.findtext('pose'), sep=' ')
    for child in list(link):
        link.remove(child)
    for item in boxes:
        xyz = item['xyz'].copy()
        if item['id'] != 'aluminum_rail_envelope':
            xyz[2] += .0003
        add_box(link, item['id'], xyz, item['size'], item['yaw'])
    delta_yaw = target_yaw-original_cfg['target_yaw']
    offset = old_board_pose[:2]-np.array(original_cfg['targets']['T0'][:2])
    center = T0[:2]+Rotation.from_euler('z', delta_yaw).as_matrix()[:2, :2]@offset
    board.find('pose').text = numbers([*center, z+.00015, 0, 0, old_board_pose[5]+delta_yaw])
    link.append(board)
    R = Rotation.from_euler('z', target_yaw).as_matrix()
    for index, (xy, size) in enumerate([([0, -.05], [.2, .001, .0001]), ([0, .05], [.2, .001, .0001]),
                                       ([-.1, 0], [.001, .1, .0001]), ([.1, 0], [.001, .1, .0001])]):
        add_box(link, f'T0_outline_{index}', T0+R@np.array([*xy, .0001]), size, target_yaw, '.02 .02 .02 1', False)
    for name, xyz in positions.items():
        world.find(f"model[@name='battery_{name}']/pose").text = numbers([*xyz, 0, 0, object_yaws[name]])
    for name in ['B', 'C', 'D']:
        world.remove(world.find(f"model[@name='battery_{name}']"))
    camera = world.find("model[@name='overview']")
    optical_from_gazebo = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]])
    camera.find('pose').text = numbers([*X[:3, 3], *Rotation.from_matrix(X[:3, :3]@optical_from_gazebo).as_euler('xyz')])
    ET.indent(tree)
    tree.write(output/'world.sdf', encoding='utf-8', xml_declaration=True)
    (output/'scenario.json').write_text(json.dumps(cfg, indent=2, ensure_ascii=False)+'\n')
    (output/'original_scenario.json').write_text(json.dumps(original_cfg, indent=2, ensure_ascii=False)+'\n')
    (output/'reviewed_points.json').write_text(json.dumps({'source': source, 'target': target}, indent=2)+'\n')
    state['camera_capture_dir'] = str(capture)
    state['association_note'] = 'Current image was acquired after read-only state sample; unchanged PowerOff joint pose, not hardware synchronized'
    (output/'captured_state.json').write_text(json.dumps(state, indent=2)+'\n')
    return dict(scenario=str(output/'scenario.json'), source=positions['A'], target=T0.tolist(),
                insertion_tcp_height_mm=56.5, nominal_lip_to_bar_clearance_mm=4.,
                nominal_plate_to_body_clearance_mm=4., physical_coordinates_verified=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--original-inputs', type=Path, required=True)
    parser.add_argument('--audit', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(build(args.original_inputs, args.audit, args.output), indent=2))
