import sys,json,time,math
sys.path.insert(0,'/home/robot/meituan_task_one_20261014_measured')
import run_hardware as player
sys.path.insert(0,str(player.HARDWARE))
from j6_45 import Robot45
from telemetry import quiet_native_auth,Telemetry
credentials=json.loads(sys.stdin.readline())
manifest=json.loads((player.ROOT/'manifest.json').read_text())
robot=None;stream=None
with player.motion_lock(player.HARDWARE/'.motion.lock'):
 try:
  with quiet_native_auth():robot=Robot45(manifest,**credentials)
  player.configuration_check(robot,manifest)
  s=robot.snapshot();remaining=robot.motion.getMotionLeftTime(s['exec_id']);queued=robot.motion.getTrajectoryQueueSize()
  print(json.dumps({'event':'before_startup','snapshot':s,'motion_left_time':remaining,'trajectory_queue_size':queued,'is_steady':robot.state.isSteady()}),flush=True)
  if s['robot_mode']=='Running':
   print(json.dumps({'event':'already_running_no_request'}),flush=True)
  else:
   assert s['robot_mode']=='Idle' and s['power_on'] and s['safety_mode']=='Normal'
   assert s['runtime']=='Stopped' and not robot.runtime.isTaskAlive(0) and s['queue']==0
   assert queued==0 and remaining<=0, 'Outstanding motion; no startup request'
   assert robot.state.isSteady() and max(map(abs,s['qd']))<math.radians(.02)
   for key in ['simulation','freedrive','servo','collision','force_control','soft_float','backdrive']:assert s[key] is False,key
   stream=Telemetry(manifest['host'],credentials['username'],credentials['password'])
   rt=stream.latest()
   assert max(abs(a-b) for a,b in zip(rt['q'],rt['target_q']))<math.radians(.02)
   assert max(map(abs,rt['target_qd']))<math.radians(.02)
   print(json.dumps({'event':'startup_intent','user_confirmed_onsite_ready':True}),flush=True)
   response=robot.manage.startup()
   print(json.dumps({'event':'startup_response','return_code':response}),flush=True)
   assert response==0,'Startup rejected; no retry'
   deadline=time.monotonic()+15
   while time.monotonic()<deadline:
    s=robot.snapshot();print(json.dumps({'event':'startup_feedback','snapshot':s}),flush=True)
    assert s['safety_mode']=='Normal','Safety state changed'
    if s['robot_mode']=='Running' and s['joint_states']==['Running']*6:break
    time.sleep(.25)
   assert s['robot_mode']=='Running','Startup did not complete; no retry'
  print(json.dumps({'event':'startup_complete','snapshot':s}),flush=True)
 finally:
  if stream:stream.close()
  if robot:robot.close()
