"""Preserve measured checkpoints, especially the two successful manual corrections."""
from datetime import datetime, timezone
import csv
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
CONT=HERE/'continuation_aligned_01'
sys.path.insert(0,str(ROOT/'scripts/task1_real_scene'))
from fit_kinematics import origins, forward, poses_to_matrices, errors

cfg=json.loads((CONT/'scenario.json').read_text())
model=origins(ET.parse(cfg['arm_urdf']).getroot())
tool_mount=Rotation.from_euler('z',90+cfg['tool_mount_yaw_deg'],degrees=True).as_matrix()
rows=[]

def add(identifier,label,q,time,source,commanded=None,controller_pose=None):
 F=forward(model,[q])[0]
 T=F[:3,:3]@tool_mount
 row=dict(id=identifier,name_zh=label,utc=time,source=str(source.resolve()),
  source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),q_rad=q,q_deg=np.rad2deg(q).tolist(),
  flange_base_xyz_mm=(F[:3,3]*1000).tolist(),flange_base_rpy_rad=Rotation.from_matrix(F[:3,:3]).as_euler('xyz').tolist(),
  nominal_tool_tcp_base_xyz_mm=((F[:3,3]+T@np.array([.045,0,.1105]))*1000).tolist(),
  nominal_tongue_front_base_xyz_mm=((F[:3,3]+T@np.array([.075,0,.1105]))*1000).tolist(),
  commanded_q_rad=commanded,controller_flange_pose=controller_pose)
 if controller_pose is not None:row['fk_controller_consistency']=errors(np.array([F]),poses_to_matrices([controller_pose]))
 rows.append(row)

def captured(identifier,label,path,commanded=None):
 s=json.loads(path.read_text());x=s['samples'][-1]
 add(identifier,label,x['q'],s['created_utc'],path,commanded,x['tcp'])

def stages(folder):
 path=next(folder.glob('*.jsonl'));r=[json.loads(x) for x in path.read_text().splitlines()]
 assert r[-1]['event']=='completed' and not any(x['event']=='failed' for x in r)
 return path,[x for x in r if x['event']=='stage_complete']

path,r=stages(HERE/'alignment_logs');add('K00','起点衔接完成',r[-1]['final_q'],r[-1]['utc'],path)
path,r=stages(HERE/'approach_logs');add('K01','原接近终点：现场发现向 +Y 偏',r[-1]['final_q'],r[-1]['utc'],path)
captured('K02','J6 微转后',HERE/'state_after_j6_adjustment.json')
captured('K03','再沿基座 -Y 平移 3 mm 后：成功插入起点',HERE/'state_after_y_translation.json')
segments=json.loads((CONT/'saved_trajectory/task_one_trajectory.json').read_text())['segments']
path,r=stages(CONT/'engage_testlift_logs');add('K04','水平插入完成',r[0]['final_q'],r[0]['utc'],path,segments[1]['points'][-1]['positions'])
captured('K05','试提完成并静止：用户确认继续',CONT/'state_after_test_lift.json',segments[2]['points'][-1]['positions'])
path,r=stages(CONT/'carry_place_logs')
labels=['抬升至搬运高度','到达 T0 上方','接近落位','落座接触','卸载承托','水平退出舌片']
for i,label in enumerate(labels):add('K'+str(i+6).zfill(2),label,r[i]['final_q'],r[i]['utc'],path,segments[i+3]['points'][-1]['positions'])
captured('K12','退出并抬高：整项流程结束',CONT/'state_after_completion.json',segments[-1]['points'][-1]['positions'])
out=HERE/'successful_keypoints';out.mkdir(exist_ok=True)
report=dict(created_utc=datetime.now(timezone.utc).isoformat(),user_confirmation='这次成功了',
 frame='physical robot base_link',joint_units='rad and deg',xyz_units='mm',
 configuration='Original V1 44 mm tongue; tool_mount_yaw_deg=0 relative to ee_link; flange-to-ee Rz90 deg',
 coordinate_semantics='Controller configured TCP offset is zero, so controller TCP means flange. Nominal tool TCP and tongue front are derived CAD points, not independently measured points.',
 source_alignment=cfg['source_anchor'],target_T0_base_xyz_mm=(np.array(cfg['targets']['T0'])*1000).tolist(),
 important_reference='K03 is the accepted insertion start. Use the successful continuation commands from that pose; do not apply J6 or -Y corrections a second time.',
 keypoints=rows)
(out/'keypoints.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
with (out/'keypoints.csv').open('w',newline='') as f:
 w=csv.writer(f);w.writerow(['id','name_zh','utc']+[f'J{i}_deg' for i in range(1,7)]+['flange_x_mm','flange_y_mm','flange_z_mm','tool_tcp_x_mm','tool_tcp_y_mm','tool_tcp_z_mm'])
 for r in rows:w.writerow([r['id'],r['name_zh'],r['utc'],*r['q_deg'],*r['flange_base_xyz_mm'],*r['nominal_tool_tcp_base_xyz_mm']])
md=['# 成功流程关键点（2026-10-14）','',
 '本次已实机完成红电池 → T0，现场用户确认成功。下面保存的是控制柜反馈关节角及其运动学坐标。坐标系为机械臂基座；位置单位 mm，关节单位 °。',
 '', '## 中途修正：下一次直接到 K03，不重复补偿','',
 '- K01：原接近终点，现场发现舌片向基座 +Y 偏、预计会碰立柱。',
 '- K01 → K02：J6 指令 -0.765899°，反馈约 -0.770801°。名义舌片前端沿基座 -Y 约 1.007 mm。',
 '- K02 → K03：保持朝向，基座 -Y 平移 3 mm；控制柜反馈约 -3.004 mm。',
 '- K03：现场回复“肉眼ok，继续”，之后插入、试提、搬运和落位均成功。这是完整流程应使用的插入起点。',
 '', '## 姿态与位置','',
 '控制柜 TCP 偏置为零，故下表“法兰”就是控制柜报告的 TCP。“工具工作点”是名义 CAD 的 `[45, 0, 110.5]` mm 经安装变换计算得到；不能当作外部测量精度。JSON 另存舌片前端 `[75, 0, 110.5]` mm 坐标与全部原始来源。',
 '', '| 编号 | 含义 | J1—J6（°） | 法兰 XYZ（mm） | 名义工具工作点 XYZ（mm） |', '|---|---|---|---|---|']
fmt=lambda xs:', '.join(f'{v:.6f}' for v in xs)
for r in rows:md.append(f"| {r['id']} | {r['name_zh']} | `{fmt(r['q_deg'])}` | `{fmt(r['flange_base_xyz_mm'])}` | `{fmt(r['nominal_tool_tcp_base_xyz_mm'])}` |")
md+=['','## 轨迹与证据','',
 '[完整关键点 JSON](keypoints.json) · [CSV](keypoints.csv)',
 '', '[成功续接的完整仿真轨迹](../continuation_aligned_01/saved_trajectory/task_one_trajectory.json)：仿真首段为 0.5 秒原地保持，实机执行后续 9 段，保留全部路点、速度、加速度和时间。关键点不是把整条曲线缩成若干直线的替代品。',
 '', '[实机录像](../continuation_aligned_01/physical_observation.mp4) · [1 倍速仿真录像](../continuation_aligned_01/replay/overview_realtime.mp4)',
 '', '下一次的一次完整流程需加入从当前退出姿态到 K03 的接近路径，并原样复用成功的插入至退出 9 段；重新仿真验证后执行。电池需恢复到记录的起抓位置与方向。',
 '', '退出状态订阅时出现 RTDE 超时，但日志均已记录 completed；随后独立读取证实关节静止、队列为空。没有重发已执行动作。']
(out/'KEYPOINTS.md').write_text('\n'.join(md)+'\n')
print(json.dumps({'count':len(rows),'path':str(out),'canonical_start':rows[3]},indent=2,ensure_ascii=False))
