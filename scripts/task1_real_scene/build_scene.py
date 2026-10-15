#!/usr/bin/env python3
"""Build task 1 from the user's fixed handle anchor and corrected T0 surface.

world == base_link. Original measurements are never overwritten. The 2.155 mm
support-height difference is represented explicitly as a local T0 support patch.
"""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from fit_kinematics import JOINTS


def numbers(values):
    return ' '.join(format(float(v), '.17g') for v in values)


def add_box(link, name, xyz, size, yaw=0., color='.72 .72 .72 1', collision=True):
    for tag in ['visual'] + (['collision'] if collision else []):
        node = ET.SubElement(link, tag, name=('visual_' if tag == 'visual' else '')+name)
        ET.SubElement(node, 'pose').text = numbers([*xyz, 0, 0, yaw])
        ET.SubElement(ET.SubElement(ET.SubElement(node, 'geometry'), 'box'), 'size').text = numbers(size)
        if tag == 'visual':
            material = ET.SubElement(node, 'material')
            ET.SubElement(material, 'ambient').text = color
            ET.SubElement(material, 'diffuse').text = color
        else:
            ode = ET.SubElement(ET.SubElement(ET.SubElement(node, 'surface'), 'friction'), 'ode')
            ET.SubElement(ode, 'mu').text = '.8'
            ET.SubElement(ode, 'mu2').text = '.8'


def build(workspace, inputs, measurements):
    share = workspace/'src/mtc_simulation'
    cfg = json.loads((share/'config/basic.json').read_text())
    state = json.loads((inputs/'captured_state.json').read_text())
    mount = json.loads((inputs/'mount_identification.json').read_text())
    red = json.loads((measurements/'point_result.json').read_text())
    target = json.loads((measurements/'placement_area_105534/point_result.json').read_text())
    meta = json.loads((measurements/'camera.json').read_text())
    anchor = np.array(red['base_xyz_m'])
    T0 = np.array(target['base_xyz_m']) + [0, 0, -.020]
    source_z = anchor[2] - cfg['battery']['dimensions'][2]
    height = T0[2] - source_z
    if height < 0 or height > .01:
        raise ValueError('Local T0 patch must be between 0 and 10 mm above source support')
    X = np.array(red['T_base_color_optical'])
    intr = meta['color_intrinsics']
    K = np.array([[intr['fx'], 0, intr['ppx']], [0, intr['fy'], intr['ppy']], [0, 0, 1.]])

    def ray_plane(uv, z):
        direction = X[:3, :3] @ np.linalg.solve(K, [*uv, 1])
        return X[:3, 3] + (z-X[2, 3])/direction[2]*direction

    image = cv2.imread(red['reference_image'])
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 110, 50), (10, 255, 255)) | cv2.inRange(hsv, (170, 110, 50), (179, 255, 255))
    edge_u = np.arange(866, 920)
    edge_v = np.array([np.flatnonzero(mask[479:500, u])[0]+479 for u in edge_u])
    fit = np.polyfit(edge_u, edge_v, 1)
    endpoints = [(870., float(np.polyval(fit, 870))), (915., float(np.polyval(fit, 915)))]
    direction = ray_plane(endpoints[1], anchor[2])-ray_plane(endpoints[0], anchor[2])
    battery_yaw = float(np.arctan2(direction[1], direction[0]))
    corners = target['corners_uv_tl_tr_br_bl']
    direction = ray_plane(corners[1], target['base_xyz_m'][2])-ray_plane(corners[0], target['base_xyz_m'][2])
    target_yaw = float(np.arctan2(direction[1], direction[0]))
    acquire = [-math.sin(battery_yaw), math.cos(battery_yaw)]
    deliver = [-math.sin(target_yaw), math.cos(target_yaw)]
    positions = {'A': [float(anchor[0]), float(anchor[1]), float(source_z)]}
    # Visible handle centers of non-target objects. Their depths are deliberately
    # inferred from the user's fixed common height, not white-board depth values.
    other_pixels = {'B': [462, 446], 'C': [1094, 461], 'D': [455, 477]}
    for name, uv in other_pixels.items():
        xyz = ray_plane(uv, anchor[2])
        positions[name] = [float(xyz[0]), float(xyz[1]), float(source_z)]
    scene_boxes = [
        dict(id='table', xyz=[.20, .0, float(source_z-.025-.0003)], size=[.80, 1.40, .05], yaw=0.),
        dict(id='T0_support_patch', xyz=[float(T0[0]), float(T0[1]), float(source_z+height/2-.0003)], size=[.2, .1, float(height)], yaw=target_yaw),
        dict(id='base_pedestal', xyz=[0., 0., float(source_z/2-.0003)], size=[.18, .18, float(-source_z)], yaw=0.),
        dict(id='aluminum_rail_envelope', xyz=[.075, 0., -.035], size=[.05, 1.3, .06], yaw=0.),
    ]
    cfg.update(task='basic', table_z=float(source_z), base_world_z=0.,
               frame_convention='world equals physical base_link; metres and radians; no display-frame rotation',
               arm_urdf=str((inputs/'controller_matched_arm.urdf').resolve()),
               world_template=str((inputs/'measured_world.sdf').resolve()),
               initial_objects=positions, yaw=battery_yaw,
               initial_joints=dict(zip(JOINTS, state['samples'][-1]['q'])),
               controller_joint_limits_rad=state['limits'],
               planning_joint_margin_rad=math.radians(2.5),
               targets={'T0': T0.tolist()}, target_yaw=target_yaw,
               tool_mount_yaw_deg=mount['tool_mount_yaw_deg'],
               acquire_directions={key: acquire for key in positions},
               delivery_direction={'left': deliver, 'right': deliver},
               transport_tcp_z=float(source_z+.35), approach_overhead_tcp_z=.20, scene_boxes=scene_boxes,
               lift_stability_s=2.0, target_footprint_size_m=[.2, .1],
               source_anchor=dict(base_xyz_m=anchor.tolist(), semantics='visible top surface of red crossbar',
                                  status='USER_FIXED_REFERENCE_FOR_THIS_SCENE'),
               target_adjustment=dict(original_base_xyz_m=target['base_xyz_m'], delta_base_m=[0, 0, -.020],
                                      corrected_base_xyz_m=T0.tolist(), source='explicit user instruction'),
               surface_model=dict(source_support_z_m=float(source_z), target_support_z_m=float(T0[2]),
                                  local_height_difference_mm=float(height*1000),
                                  interpretation='two level local support surfaces; uneven board modeled by T0 patch'),
               recognized_orientation=dict(red_bar_yaw_deg=math.degrees(battery_yaw),
                                           target_long_axis_yaw_deg=math.degrees(target_yaw),
                                           red_edge_pixels=endpoints),
               obstacle_positions_source=dict(pixels=other_pixels, method='RGB selection and anchor-height ray-plane intersection',
                                              rail='Conservative envelope around observed front aluminum cross rail; not a full room scan'))
    cfg['assumptions'].update(perception='scene anchored to real RGB-D measurements; Gazebo truth used only for simulation feedback',
                              tool='original V1 44 mm confirmed by user; mount yaw identified from current image; nominal CAD TCP',
                              arm_control='controller-matched kinematic model; Gazebo position servo; physical dynamics unvalidated',
                              camera='external D435i pose from saved hand-eye calibration',
                              obstacle_geometry='non-target battery centers estimated from image; rail and pedestal conservative envelopes')
    cfg['provenance'] = {}
    for path in [measurements/'point_result.json', measurements/'placement_area_105534/point_result.json',
                 Path(red['calibration_file']), inputs/'captured_state.json', inputs/'mount_identification.json',
                 inputs/'kinematics_reference.json', inputs/'kinematics_validation.json', inputs/'controller_matched_arm.urdf']:
        cfg['provenance'][str(path.resolve())] = hashlib.sha256(path.read_bytes()).hexdigest()

    tree = ET.parse(share/'worlds/basic.sdf')
    world = tree.getroot().find('world')
    ET.SubElement(world.find("model[@name='floor']"), 'pose').text = '0 0 -1 0 0 0'
    link = world.find("model[@name='table']/link")
    board = copy.deepcopy(link.find("visual[@name='visual_original_board']"))
    for child in list(link):
        link.remove(child)
    for item in scene_boxes:
        xyz = item['xyz'].copy()
        if item['id'] != 'aluminum_rail_envelope':
            xyz[2] += .0003  # planner-only surface inset is absent from physical geometry
        add_box(link, item['id'], xyz, item['size'], item['yaw'],
                '.95 .95 .95 1' if item['id']=='T0_support_patch' else '.65 .64 .58 1')
    delta_yaw = target_yaw-math.pi/2
    center = T0[:2] + Rotation.from_euler('z', delta_yaw).as_matrix()[:2, :2] @ (np.array([.16964744013519964, .04300115410517032])-np.array([.299848413, -.115038402]))
    board.find('pose').text = numbers([*center, source_z+.00015, 0, 0, -math.pi/2+delta_yaw])
    link.append(board)
    rot = Rotation.from_euler('z', target_yaw).as_matrix()
    for index, (xy, size) in enumerate([([0, -.05], [.2, .001, .0001]), ([0, .05], [.2, .001, .0001]),
                                        ([-.1, 0], [.001, .1, .0001]), ([.1, 0], [.001, .1, .0001])]):
        center = T0 + rot @ np.array([*xy, .0001])
        add_box(link, f'T0_outline_{index}', center, size, target_yaw, '.02 .02 .02 1', False)
    for name, xyz in positions.items():
        world.find(f"model[@name='battery_{name}']/pose").text = numbers([*xyz, 0, 0, battery_yaw])
        body=world.find(f"model[@name='battery_{name}']/link/collision[@name='part_0']")
        geometry=body.find('geometry');geometry.clear()
        ET.SubElement(ET.SubElement(geometry,'box'),'size').text='.07 .08 .05'
        ET.SubElement(body,'pose').text='0 0 .025 0 0 0'
    cfg['assumptions']['battery_body_collision']='Conservative 70x80x50 mm box enclosing original lower-body mesh; original handle collision boxes, mass and inertia retained'
    camera = world.find("model[@name='overview']")
    optical_from_gazebo = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]])
    rpy = Rotation.from_matrix(X[:3, :3] @ optical_from_gazebo).as_euler('xyz')
    camera.find('pose').text = numbers([*X[:3, 3], *rpy])
    camera.find('link/sensor/camera/horizontal_fov').text = str(2*math.atan(intr['width']/(2*intr['fx'])))
    camera.find('link/sensor/update_rate').text='5'
    camera.find('link/sensor/camera/image/width').text='640'
    camera.find('link/sensor/camera/image/height').text='360'
    ET.indent(tree)
    tree.write(inputs/'measured_world.sdf', encoding='utf-8', xml_declaration=True)
    (inputs/'scenario.json').write_text(json.dumps(cfg, indent=2, ensure_ascii=False)+'\n')
    return {'scenario': str(inputs/'scenario.json'), 'anchor': anchor.tolist(), 'T0': T0.tolist(),
            'source_support_z': source_z, 'local_T0_height_mm': height*1000,
            'red_bar_yaw_deg': math.degrees(battery_yaw), 'tool_mount_yaw_deg': cfg['tool_mount_yaw_deg']}


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--workspace', type=Path, default=Path(__file__).resolve().parents[2])
    p.add_argument('--inputs', type=Path, required=True)
    p.add_argument('--measurements', type=Path, required=True)
    args = p.parse_args()
    print(json.dumps(build(args.workspace, args.inputs, args.measurements), indent=2))
