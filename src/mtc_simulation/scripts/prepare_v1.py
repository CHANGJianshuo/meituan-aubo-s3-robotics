#!/usr/bin/env python3
"""Generate relocatable V1 URDF from the preserved S3 model and shared tool CAD dimensions."""
import argparse
import json
import math
import uuid
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_share_directory

JOINTS = ['shoulder_joint', 'upperArm_joint', 'foreArm_joint', 'wrist1_joint', 'wrist2_joint', 'wrist3_joint']

def prepare(share, out, task, scenario_file=None):
    out.mkdir(parents=True, exist_ok=True)
    config = json.loads((Path(scenario_file) if scenario_file else share / 'config' / f'{task}.json').read_text())
    if config['task'] != task:
        raise ValueError('Scenario task does not match requested task')
    mount_yaw=math.radians(config.get('tool_mount_yaw_deg',90.0))
    root = ET.parse(config.get('arm_urdf', share / 'assets/arm.urdf')).getroot()
    # Intersection used by the team's reviewed S3 planning/execution adapter.
    bounds=dict(zip(JOINTS,[2*math.pi,math.pi,2.41,2.66,2*math.pi,2.23]))
    limits = config.get('controller_joint_limits_rad')
    selected_limits = {}
    for joint in root.findall('joint'):
        if joint.get('name') in bounds:
            limit=joint.find('limit');b=bounds[joint.get('name')]
            lower, upper = (-b, b) if limits is None else (limits[0][JOINTS.index(joint.get('name'))], limits[1][JOINTS.index(joint.get('name'))])
            lower, upper = max(float(limit.get('lower')), lower), min(float(limit.get('upper')), upper)
            margin = float(config.get('planning_joint_margin_rad', 0.))
            lower += margin; upper -= margin
            if not lower < config['initial_joints'][joint.get('name')] < upper:
                raise ValueError('Initial joint outside selected limits')
            limit.set('lower',str(lower)); limit.set('upper',str(upper))
            selected_limits[joint.get('name')] = (lower, upper)
    for mesh in root.iter('mesh'):
        mesh.set('filename', mesh.get('filename').replace('model://mtc_simulation/', share.as_uri() + '/'))
    def append(xml):
        node = ET.fromstring(xml); root.append(node); return node
    append('<link name="world"/>')
    base_z = float(config.get('base_world_z', .651))
    append(f'<joint name="world_mount" type="fixed"><parent link="world"/><child link="base_link"/><origin xyz="0 0 {base_z}"/></joint>')
    append('<link name="ee_link"/>')
    append(f'<joint name="ee_fixed" type="fixed"><parent link="wrist3_Link"/><child link="ee_link"/><origin rpy="0 0 {math.pi/2}"/></joint>')
    tool = append('''<link name="v1_hook"><inertial><origin xyz=".015 0 .060"/>
      <mass value="0.25"/><inertia ixx="0.0005" iyy="0.0005" izz="0.0002" ixy="0" ixz="0" iyz="0"/></inertial></link>''')
    # CAD nominal geometry, same transform and dimensions as planner.yaml/tool_model.cpp.
    bodies = [('disc','0 0 .0005','<cylinder radius=".0375" length=".015"/>'),
              ('stem','0 0 .058','<box size=".020 .030 .100"/>'),
              ('tongue','.0325 0 .1105','<box size=".085 .044 .005"/>'),
              ('lip','.0725 0 .1055','<box size=".005 .044 .005"/>')]
    for name, xyz, geometry in bodies:
        for tag in ('visual','collision'):
            el=ET.SubElement(tool,tag,name='v1_'+name+'_'+tag)
            ET.SubElement(el,'origin',xyz=xyz)
            geom=ET.SubElement(el,'geometry');geom.append(ET.fromstring(geometry))
            if tag=='visual':
                mat=ET.SubElement(el,'material',name='steel');ET.SubElement(mat,'color',rgba='.75 .77 .80 1')
    append(f'<joint name="v1_mount" type="fixed"><parent link="ee_link"/><child link="v1_hook"/><origin rpy="0 0 {mount_yaw}"/></joint>')
    append('''<link name="tcp"><inertial><mass value=".001"/><inertia ixx="1e-8" iyy="1e-8" izz="1e-8" ixy="0" ixz="0" iyz="0"/></inertial></link>''')
    append(f'<joint name="tcp_joint" type="fixed"><parent link="v1_hook"/><child link="tcp"/><origin xyz=".045 0 .1105" rpy="0 {math.pi/2} 0"/></joint>')
    for joint in ('v1_mount','tcp_joint'):
        append(f'<gazebo reference="{joint}"><preserveFixedJoint>true</preserveFixedJoint></gazebo>')
    append('<gazebo reference="v1_hook"><mu1>0.8</mu1><mu2>0.8</mu2></gazebo>')
    append('<gazebo><self_collide>true</self_collide></gazebo>')
    control=append('<ros2_control name="V1GazeboSystem" type="system"><hardware><plugin>gz_ros2_control/GazeboSimSystem</plugin></hardware></ros2_control>')
    for name in JOINTS:
        j=ET.SubElement(control,'joint',name=name)
        command=ET.SubElement(j,'command_interface',name='position')
        ET.SubElement(command,'param',name='min').text=str(selected_limits[name][0])
        ET.SubElement(command,'param',name='max').text=str(selected_limits[name][1])
        p=ET.SubElement(j,'state_interface',name='position')
        ET.SubElement(p,'param',name='initial_value').text=str(config['initial_joints'][name])
        ET.SubElement(j,'state_interface',name='velocity')
        ET.SubElement(j,'state_interface',name='effort')
    append(f'''<gazebo><plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
      <parameters>{out / 'controllers.yaml'}</parameters><position_proportional_gain>0.5</position_proportional_gain>
    </plugin></gazebo>''')
    ET.indent(root);ET.ElementTree(root).write(out/'robot.urdf',encoding='utf-8',xml_declaration=True)
    controllers={'controller_manager':{'ros__parameters':{'update_rate':250,'use_sim_time':True,'enforce_command_limits':True,
       'joint_state_broadcaster':{'type':'joint_state_broadcaster/JointStateBroadcaster'},
       'arm_controller':{'type':'joint_trajectory_controller/JointTrajectoryController'}}},
       'arm_controller':{'ros__parameters':{'joints':JOINTS,'command_interfaces':['position'],
       'state_interfaces':['position','velocity'],'allow_partial_joints_goal':False,
       'constraints':{'goal_time':2.0,'stopped_velocity_tolerance':0.02,
                      **{j:{'trajectory':0.08,'goal':0.005} for j in JOINTS}}}}}
    (out/'controllers.yaml').write_text(yaml.safe_dump(controllers))
    srdf=ET.parse(Path(get_package_share_directory('aubo_moveit_config'))/'config/aubo_robot.srdf')
    for node in list(srdf.getroot().findall('virtual_joint')):srdf.getroot().remove(node)
    for name in ('ee_link','wrist3_Link'):
        ET.SubElement(srdf.getroot(),'disable_collisions',link1='v1_hook',link2=name,reason='Adjacent')
    srdf.write(out/'robot.srdf',encoding='utf-8',xml_declaration=True)
    config['tool_type']='passive_hook_v1'
    config['simulation_only']=True
    config['tool_mass_kg_assumed']=0.25
    config['base_world_z']=base_z
    config['session_id']=uuid.uuid4().hex
    world=ET.parse(config.get('world_template', share/'worlds'/f'{task}.sdf'))
    observer=world.getroot().find('world/plugin[@name="mtc::V1Observer"]')
    ET.SubElement(observer,'session_id').text=config['session_id']
    ET.indent(world);world.write(out/'world.sdf',encoding='utf-8',xml_declaration=True)
    (out/'scenario.json').write_text(json.dumps(config,indent=2)+'\n')
    return config

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--task',choices=['basic','sequence'],default='basic');p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();prepare(Path(get_package_share_directory('mtc_simulation')),a.output.resolve(),a.task)
