"""Rebuild source alignment from the operator-approved tool pose; no robot I/O."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'scripts/task1_real_scene'))
from fit_kinematics import origins, forward

state_path=HERE/'state_after_y_translation.json'
state=json.loads(state_path.read_text())
cfg=json.loads((HERE/'y_translation_01/scenario.json').read_text())
q=state['samples'][-1]['q']
F=forward(origins(ET.parse(cfg['arm_urdf']).getroot()),[q])[0]
tool_rotation=F[:3,:3]@Rotation.from_euler('z',90+cfg['tool_mount_yaw_deg'],degrees=True).as_matrix()
tcp=F[:3,3]+tool_rotation@np.array([.045,0,.1105])
direction=tool_rotation[:,0].copy(); direction[2]=0; direction/=np.linalg.norm(direction)
backoff=cfg['approach_backoff_m']['right']
old_source=np.array(cfg['initial_objects']['A'])
source=old_source.copy(); source[:2]=tcp[:2]+backoff*direction[:2]
yaw=float((np.arctan2(direction[1],direction[0])-np.pi/2)%(2*np.pi))
old_yaw=cfg['object_yaws']['A']
assert np.linalg.norm(source-old_source)<.01 and abs(yaw-old_yaw)<np.deg2rad(2)
assert abs(tcp[2]-(source[2]+cfg['insertion_tcp_height_above_support_m']))<.0001
out=HERE/'continuation_aligned_01';out.mkdir()
cfg['initial_joints']=dict(zip(cfg['initial_joints'],q))
cfg['initial_objects']['A']=source.tolist()
cfg['object_yaws']['A']=yaw;cfg['yaw']=yaw
cfg['acquire_directions']['A']=direction[:2].tolist()
cfg['source_anchor']=dict(base_xyz_m=(source+[0,0,.080]).tolist(),
 semantics='geometric top surface centre of crossbar',
 status='SOURCE_ALIGNMENT_ESTIMATE_FROM_OPERATOR_APPROVED_PREINSERTION_TOOL_POSE')
cfg.pop('approach_overhead_tcp_z',None)
cfg['world_template']=str(out/'world.sdf')
cfg['assumptions']['perception']='Source XY and yaw inferred from operator-approved preinsertion alignment, nominal 60 mm backoff and tool CAD; height and target retained. Not a new hand-eye calibration.'
world=ET.parse(HERE.parent/'inputs_red_only/world.sdf')
world.getroot().find("world/model[@name='battery_A']/pose").text=' '.join(format(float(v),'.17g') for v in [*source,0,0,yaw])
ET.indent(world);world.write(out/'world.sdf',encoding='utf-8',xml_declaration=True)
report=dict(created_utc=datetime.now(timezone.utc).isoformat(),user_confirmation='肉眼ok，继续',
 interpretation='Operator accepted lateral alignment after J6 adjustment and base -Y 3 mm translation',
 source_state=str(state_path),source_state_sha256=hashlib.sha256(state_path.read_bytes()).hexdigest(),
 current_nominal_tool_tcp_base_mm=(tcp*1000).tolist(),current_tool_direction_base=direction.tolist(),
 prior_source_base_mm=(old_source*1000).tolist(),new_source_base_mm=(source*1000).tolist(),
 inferred_source_change_mm=((source-old_source)*1000).tolist(),source_yaw_deg=float(np.rad2deg(yaw)),
 source_yaw_change_deg=float(np.rad2deg(yaw-old_yaw)),unchanged_target=cfg['targets'],
 unchanged_support_z_m=cfg['table_z'],insertion_distance_m=backoff,
 physical_measurement_precision_verified=False,scope='Validate new continuation; execute insertion and test lift first, then observe attachment',
 pending='Simulation validation')
for name,value in [('scenario.json',cfg),('alignment_reference.json',report)]:
 (out/name).write_text(json.dumps(value,indent=2,ensure_ascii=False)+'\n')
print(json.dumps(report,indent=2,ensure_ascii=False))
