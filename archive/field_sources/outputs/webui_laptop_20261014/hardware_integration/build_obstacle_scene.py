"""Relative obstacle estimate anchored to the physically proven red source pose.

RGB-D registration provides current view orientation. Intersect colored bar-top
pixels with the common 80 mm top plane, then anchor red to its successful pose.
Other colors get a 6 mm per-side inflated solid envelope. Not a new calibration.
"""
import json,copy
from pathlib import Path
import xml.etree.ElementTree as ET
import cv2,numpy as np
HERE=Path(__file__).resolve().parent
WS=HERE.parents[2]
ATT=WS/'outputs/task1_real_scene_20261014/revision_01/physical_attempt_20261014T184851Z'
scenario=json.loads((ATT/'half_time_01/scenario.json').read_text())
capture=Path((HERE/'capture_path.txt').read_text())
meta=json.loads((capture/'capture.json').read_text())
i=meta['camera']['color_intrinsics'];K=np.array([[i['fx'],0,i['ppx']],[0,i['fy'],i['ppy']],[0,0,1.]])
T=np.array(json.loads((HERE/'camera_registration_candidate.json').read_text())['T_base_current_camera_candidate'])
z=scenario['table_z']+.080
pixels={};points={}
for color in ['red','yellow','green','blue']:
 m=cv2.imread(str(HERE/(color+'_mask.png')),0)>0
 y,x=np.nonzero(m);top=y.min();near=(y<=top+7)
 uv=np.array([np.mean(x[near]),np.mean(y[near])]);pixels[color]=uv.tolist()
 d=T[:3,:3]@np.linalg.inv(K)@[*uv,1]
 points[color]=T[:3,3]+d*(z-T[2,3])/d[2]
anchor=np.array(scenario['initial_objects']['A']);anchor[2]=z
shift=anchor-points['red'];points={c:p+shift for c,p in points.items()}
world=ET.parse(scenario['world_template']);w=world.getroot().find('world')
obstacles=[]
for color in ['yellow','green','blue']:
 p=points[color].copy();p[2]=scenario['table_z']+.042
 size=[.082,.092,.084];yaw=scenario['yaw']
 item=dict(id='neighbor_'+color,xyz=p.tolist(),size=size,yaw=yaw)
 obstacles.append(item);scenario['scene_boxes'].append(item)
 model=ET.SubElement(w,'model',name='neighbor_'+color)
 ET.SubElement(model,'static').text='true'
 ET.SubElement(model,'pose').text=' '.join(map(str,[*p,0,0,yaw]))
 link=ET.SubElement(model,'link',name='obstacle')
 for kind in ['collision','visual']:
  node=ET.SubElement(link,kind,name='solid_envelope')
  ET.SubElement(ET.SubElement(ET.SubElement(node,'geometry'),'box'),'size').text=' '.join(map(str,size))
 sensor=ET.SubElement(link,'sensor',name='contacts',type='contact')
 ET.SubElement(sensor,'always_on').text='true'
 ET.SubElement(ET.SubElement(sensor,'contact'),'collision').text='solid_envelope'
 ET.SubElement(sensor,'update_rate').text='100'
world.write(HERE/'obstacle_world.sdf')
scenario['world_template']=str(HERE/'obstacle_world.sdf')
scenario['assumptions']['obstacle_geometry']='Observed yellow/green/blue, conservative solid envelopes inflated 6 mm per side, 4 mm higher; anchored to proven red source'
scenario['obstacle_positions_source']=dict(method=__doc__,bar_top_pixels=pixels,obstacles=obstacles)
(HERE/'obstacle_scenario.json').write_text(json.dumps(scenario,indent=2))
report=dict(method=__doc__,bar_top_pixels=pixels,base_top_points_m={c:p.tolist() for c,p in points.items()},
 relative_to_red_mm={c:((p-points['red'])*1000).tolist() for c,p in points.items()},
 envelope_inflation_per_side_mm=6,upper_height_inflation_mm=4,
 caveat='Relative image estimate; not independent millimetre metrology. Simulation includes conservative inflated neighbors.')
(HERE/'obstacle_estimate.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
