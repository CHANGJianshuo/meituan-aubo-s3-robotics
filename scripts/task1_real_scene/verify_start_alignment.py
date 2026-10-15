#!/usr/bin/env python3
"""Validate a bounded start alignment in an isolated, already running Gazebo scene.

Never connects to a hardware driver. Both the Gazebo component and per-run
observer token must match before sending the simulation trajectory.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate', type=Path, required=True)
    parser.add_argument('--scene', type=Path, required=True)
    parser.add_argument('--math-dir', type=Path, required=True)
    parser.add_argument('--domain', type=int, required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != str(args.domain):
        raise ValueError('Explicit isolated ROS domain required')
    sys.path.insert(0, str(args.math_dir))
    from trajectory import dense, validate
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data
    from control_msgs.action import FollowJointTrajectory
    from controller_manager_msgs.srv import ListHardwareComponents
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
    from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetStateValidity
    from sensor_msgs.msg import JointState
    from shape_msgs.msg import SolidPrimitive
    from std_msgs.msg import String
    from trajectory_msgs.msg import JointTrajectoryPoint

    candidate = json.loads(args.candidate.read_text())
    cfg = json.loads((args.scene/'scenario.json').read_text())
    assert cfg['simulation_only'] and cfg['tool_type'] == 'passive_hook_v1'
    stats = validate(candidate, cfg['controller_joint_limits_rad'])
    assert len(candidate['segments']) == 1
    segment = candidate['segments'][0]
    assert segment['primitive'] in ('align_to_saved_start', 'j6_tip_minus_y_1mm', 'cartesian_minus_y_3mm')
    assert max(abs(a-b) for a, b in zip(segment['points'][0]['positions'],
                                      segment['points'][-1]['positions'])) <= math.radians(1)
    cartesian_check = None
    if segment['primitive'] == 'j6_tip_minus_y_1mm':
        from fit_kinematics import forward, origins
        q0, q1 = (np.array(segment['points'][i]['positions']) for i in (0, -1))
        assert np.max(np.abs(q1[:5]-q0[:5])) < 1e-12
        model = ET.parse(args.scene/'robot.urdf').getroot()
        fk = forward(origins(model), [q0, q1])
        # Nominal V1 leading edge centre, with its confirmed flange mounting.
        local = Rotation.from_euler('z', 90+cfg['tool_mount_yaw_deg'], degrees=True).apply([.075, 0, .1105])
        positions = fk[:, :3, 3]+np.einsum('nij,j->ni', fk[:, :3, :3], local)
        delta = positions[1]-positions[0]
        assert abs(delta[1]+.001) < 1e-6 and abs(delta[2]) < .00005
        cartesian_check = dict(point='nominal tongue leading-edge centre',
                               delta_base_mm=(delta*1000).tolist(),
                               only_j6_changed=True, delta_j6_deg=float(np.rad2deg(q1[5]-q0[5])))
    elif segment['primitive'] == 'cartesian_minus_y_3mm':
        from fit_kinematics import forward, origins
        model = ET.parse(args.scene/'robot.urdf').getroot()
        _, path = dense(segment, .01)
        fk = forward(origins(model), [point[0] for point in path])
        delta = fk[-1, :3, 3]-fk[0, :3, 3]
        angles = Rotation.from_matrix(np.transpose(fk[:, :3, :3], (0, 2, 1))@fk[0, :3, :3]).magnitude()
        position_error = np.max(np.abs(fk[:, [0, 2], 3]-fk[0, [0, 2], 3]))
        assert np.max(np.abs(delta-[0, -.003, 0])) < 1e-6
        assert position_error < .00005 and np.max(angles) < math.radians(.01)
        assert np.all(np.diff(fk[:, 1, 3]) <= 1e-8)
        cartesian_check = dict(point='flange; every rigid tool point translates equally when orientation is held',
                               delta_base_mm=(delta*1000).tolist(),
                               maximum_xz_deviation_mm=float(position_error*1000),
                               maximum_orientation_change_deg=float(np.rad2deg(np.max(angles))))
    names = candidate['joint_names']
    rclpy.init()
    node = rclpy.create_node('verify_start_alignment')
    observed = {}

    def joint_callback(msg):
        observed['joints'] = dict(zip(msg.name, msg.position))
        observed['joint_time'] = time.monotonic()

    def contact_callback(msg):
        observed['contacts'] = json.loads(msg.data)
        observed['contact_time'] = time.monotonic()

    subscriptions = [
        node.create_subscription(JointState, '/joint_states', joint_callback, qos_profile_sensor_data),
        node.create_subscription(String, '/simulation/contacts', contact_callback, 10),
    ]

    def future_result(future, timeout=10.):
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError('Simulation request timed out')
        return future.result()

    def service(type_, name, request):
        client = node.create_client(type_, name)
        if not client.wait_for_service(timeout_sec=20.):
            raise RuntimeError('Missing simulation service: '+name)
        return future_result(client.call_async(request))

    def guard():
        now = time.monotonic()
        assert now-observed.get('joint_time', 0) < 2
        assert now-observed.get('contact_time', 0) < 2
        contacts = observed['contacts']
        assert contacts['session_id'] == cfg['session_id']
        assert contacts['gazebo_control_plugin'] is True
        for row in contacts.get('total', {}).values():
            if row['kind'] in ('unexpected', 'tool_contact'):
                assert row['max_depth_m'] <= .001, row

    goal_handle = None
    try:
        hardware = service(ListHardwareComponents, '/controller_manager/list_hardware_components',
                           ListHardwareComponents.Request()).component
        assert len(hardware) == 1 and hardware[0].name == 'V1GazeboSystem'
        assert hardware[0].state.label == 'active'
        assert hardware[0].plugin_name in ('', 'gz_ros2_control/GazeboSimSystem')
        deadline = time.monotonic()+10
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if 'contacts' in observed and 'joints' in observed:
                break
        guard()
        assert max(abs(observed['joints'][j]-q) for j, q in
                   zip(names, segment['points'][0]['positions'])) < math.radians(.01)
        boxes = list(cfg['scene_boxes'])
        battery_parts = []
        world = ET.parse(args.scene/'world.sdf').getroot()
        for key, xyz in cfg['initial_objects'].items():
            # Use the same shell, posts and bar as Gazebo. A full 80 mm box
            # incorrectly fills the handle opening and space above the shell.
            body = world.find(f"world/model[@name='battery_{key}']/link[@name='body']")
            assert body is not None and body.find('pose') is None
            rotation = Rotation.from_euler('z', cfg.get('object_yaws', {}).get(key, cfg['yaw']))
            for shape in body.findall('collision'):
                geometry = shape.find('geometry/box/size')
                assert geometry is not None, 'Expected reviewed battery box collisions'
                local = np.fromstring(shape.findtext('pose', '0 0 0 0 0 0'), sep=' ')
                assert len(local) == 6
                identifier = 'battery_'+key+'/'+shape.get('name')
                boxes.append(dict(id=identifier, xyz=(np.array(xyz)+rotation.apply(local[:3])).tolist(),
                                  size=np.fromstring(geometry.text, sep=' ').tolist(),
                                  quaternion=(rotation*Rotation.from_euler('xyz', local[3:])).as_quat().tolist()))
                battery_parts.append(identifier)
        request = ApplyPlanningScene.Request()
        request.scene = PlanningScene(is_diff=True)
        request.scene.robot_state.is_diff = True
        # Remove the obsolete coarse proxy only when present. MoveIt rejects
        # removing unknown object IDs, including in a newly started scene.
        read_scene = GetPlanningScene.Request()
        read_scene.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        existing_ids = {obj.id for obj in service(GetPlanningScene, '/get_planning_scene', read_scene).scene.world.collision_objects}
        for key in cfg['initial_objects']:
            identifier = 'alignment/battery_'+key
            if identifier in existing_ids:
                request.scene.world.collision_objects.append(CollisionObject(
                    id=identifier, operation=CollisionObject.REMOVE))
        for box in boxes:
            obj = CollisionObject(id='alignment/'+box['id'], operation=CollisionObject.ADD)
            obj.header.frame_id = 'world'
            p = Pose()
            p.position.x, p.position.y, p.position.z = map(float, box['xyz'])
            if 'quaternion' in box:
                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = box['quaternion']
            else:
                p.orientation.z = math.sin(box.get('yaw', 0)/2)
                p.orientation.w = math.cos(box.get('yaw', 0)/2)
            obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(map(float, box['size'])))]
            obj.primitive_poses = [p]
            request.scene.world.collision_objects.append(obj)
        assert service(ApplyPlanningScene, '/apply_planning_scene', request).success
        validity = node.create_client(GetStateValidity, '/check_state_validity')
        assert validity.wait_for_service(timeout_sec=10.)
        _, samples = dense(segment, .01)
        for i, (q, _, _) in enumerate(samples):
            request = GetStateValidity.Request()
            request.robot_state.joint_state.name = names
            request.robot_state.joint_state.position = q
            response = future_result(validity.call_async(request))
            assert response.valid, f'Collision/invalid state at sample {i}: {response.contacts}'
        guard()
        client = ActionClient(node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        assert client.wait_for_server(timeout_sec=10.)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = names
        for row in segment['points']:
            p = JointTrajectoryPoint(positions=row['positions'], velocities=row['velocities'],
                                     accelerations=row['accelerations'])
            p.time_from_start.sec = int(row['t'])
            p.time_from_start.nanosec = round((row['t']-int(row['t']))*1e9)
            goal.trajectory.points.append(p)
        goal_handle = future_result(client.send_goal_async(goal))
        assert goal_handle.accepted
        future = goal_handle.get_result_async()
        deadline = time.monotonic()+20
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            guard()
        assert future.done() and future.result().status == 4
        assert future.result().result.error_code == 0
        goal_handle = None
        until = time.monotonic()+1.5
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
            guard()
        end_error = max(abs(observed['joints'][j]-q) for j, q in
                        zip(names, segment['points'][-1]['positions']))
        assert end_error < math.radians(.01)
        result = dict(passed=True, simulation_only=True, checked_samples=len(samples),
                      candidate_sha256=hashlib.sha256(args.candidate.read_bytes()).hexdigest(),
                      scene_session_id=cfg['session_id'], endpoint_error_deg=math.degrees(end_error),
                      stats=stats, unexpected_contact_over_1mm=False,
                      primitive=segment['primitive'], cartesian_check=cartesian_check,
                      battery_collision_parts=battery_parts,
                      collision_world_sha256=hashlib.sha256((args.scene/'world.sdf').read_bytes()).hexdigest(),
                      verifier_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
        (args.candidate.parent/'verification.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result, indent=2))
    finally:
        if goal_handle is not None:
            future_result(goal_handle.cancel_goal_async())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
