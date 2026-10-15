#!/usr/bin/env python3
"""Validate an unloaded two-stage return from saved, accepted approach paths in an isolated, already running Gazebo scene.

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
    from tf2_msgs.msg import TFMessage

    candidate = json.loads(args.candidate.read_text())
    cfg = json.loads((args.scene/'scenario.json').read_text())
    assert cfg['simulation_only'] and cfg['tool_type'] == 'passive_hook_v1'
    stats = validate(candidate, cfg['controller_joint_limits_rad'])
    assert candidate['task'] == 'return_to_start'
    assert [s['primitive'] for s in candidate['segments']] == ['return_via_red_approach','return_to_original_start']
    assert max(stats['peak_velocity_deg_s']) <= 6.5
    segments = candidate['segments']
    segment = segments[0]
    proof=json.loads((args.candidate.parent/'composition_validation.json').read_text())
    assert proof['passed'] and proof['candidate_sha256']==hashlib.sha256(args.candidate.read_bytes()).hexdigest()
    trace=(args.candidate.parent/'simulation_observations.jsonl').open('x')
    trace_count=0
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

    def physics_callback(msg):
        nonlocal trace_count
        poses={t.child_frame_id:dict(xyz=[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z],quat=[t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w]) for t in msg.transforms}
        observed['poses']=poses;observed['pose_time']=time.monotonic()
        trace.write(json.dumps(dict(poses=poses,wall=time.monotonic()))+'\n');trace.flush();trace_count+=1

    subscriptions = [
        node.create_subscription(JointState, '/joint_states', joint_callback, qos_profile_sensor_data),
        node.create_subscription(String, '/simulation/contacts', contact_callback, 10),
        node.create_subscription(TFMessage, '/simulation/ground_truth', physics_callback, 10),
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
        assert now-observed.get('pose_time',0)<2
        poses=observed['poses']
        for key,initial in cfg['initial_objects'].items():
            obj=poses['battery_'+key]
            assert math.dist(obj['xyz'],initial)<.003, ('battery moved',key,obj)
            assert 1-2*(obj['quat'][0]**2+obj['quat'][1]**2)>.995
        q=poses['tcp']['quat'];ref=cfg['fixed_tcp_orientation_xyzw']
        assert math.degrees(2*math.acos(min(1.,abs(sum(a*b for a,b in zip(q,ref))))))<.1
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
            if 'contacts' in observed and 'joints' in observed and 'poses' in observed:
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
        checked_samples=0
        for si, segment in enumerate(segments):
            _,samples=dense(segment,.01)
            for i,(q,_,_) in enumerate(samples):
                request=GetStateValidity.Request()
                request.robot_state.joint_state.name=names
                request.robot_state.joint_state.position=q
                response=future_result(validity.call_async(request))
                assert response.valid, f'Collision/invalid state at stage {si+1} sample {i}: {response.contacts}'
                checked_samples+=1
            guard()
        client=ActionClient(node,FollowJointTrajectory,'/arm_controller/follow_joint_trajectory')
        assert client.wait_for_server(timeout_sec=10.)
        receipts=[]
        with (args.candidate.parent/'simulation_commands.jsonl').open('x') as commands:
            for si,segment in enumerate(segments):
                assert max(abs(observed['joints'][j]-q) for j,q in zip(names,segment['points'][0]['positions']))<math.radians(.01)
                goal=FollowJointTrajectory.Goal();goal.trajectory.joint_names=names
                for row in segment['points']:
                    point=JointTrajectoryPoint(positions=row['positions'],velocities=row['velocities'],accelerations=row['accelerations'])
                    ns=round(row['t']*1e9);point.time_from_start.sec=ns//1000000000;point.time_from_start.nanosec=ns%1000000000
                    goal.trajectory.points.append(point)
                commands.write(json.dumps(segment)+'\n');commands.flush()
                goal_handle=future_result(client.send_goal_async(goal))
                assert goal_handle.accepted
                future=goal_handle.get_result_async();deadline=time.monotonic()+segment['points'][-1]['t']*4+20
                while not future.done() and time.monotonic()<deadline:
                    rclpy.spin_once(node,timeout_sec=.02);guard()
                assert future.done() and future.result().status==4 and future.result().result.error_code==0
                goal_handle=None
                until=time.monotonic()+1.
                while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05);guard()
                error=math.degrees(max(abs(observed['joints'][j]-q) for j,q in zip(names,segment['points'][-1]['positions'])))
                assert error<.01
                receipts.append(dict(stage=si+1,endpoint_error_deg=error))
        until = time.monotonic()+3.
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
            guard()
        end_error = max(abs(observed['joints'][j]-q) for j, q in
                        zip(names, segment['points'][-1]['positions']))
        assert end_error < math.radians(.01)
        result = dict(passed=True, simulation_only=True, checked_samples=checked_samples,
                      candidate_sha256=hashlib.sha256(args.candidate.read_bytes()).hexdigest(),
                      scene_session_id=cfg['session_id'], endpoint_error_deg=math.degrees(end_error),
                      stats=stats, unexpected_contact_over_1mm=False,
                      stages=receipts, actual_physics_samples=trace_count, all_four_batteries_undisturbed=True,
                      original_task_unchanged=True, contacts=observed['contacts'],
                      battery_collision_parts=battery_parts,
                      collision_world_sha256=hashlib.sha256((args.scene/'world.sdf').read_bytes()).hexdigest(),
                      verifier_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
        (args.candidate.parent/'verification.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result, indent=2))
    finally:
        if goal_handle is not None:
            future_result(goal_handle.cancel_goal_async())
        trace.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
