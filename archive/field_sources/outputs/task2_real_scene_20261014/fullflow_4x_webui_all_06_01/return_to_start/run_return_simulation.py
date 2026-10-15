#!/usr/bin/env python3
"""Own one isolated Gazebo/ROS simulation and collect evidence. Never starts a robot driver."""
import argparse
import json
import os
import re
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'src/mtc_simulation/scripts'))
from prepare_v1 import prepare
from analyze_v1 import analyze

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--task',choices=['basic','sequence'],default='basic')
    p.add_argument('--order',help='basic: one battery; sequence: three distinct letters from ABCD')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--gui',action='store_true')
    p.add_argument('--pin-moveit-plugins',action='store_true',help='opt in to the documented MoveIt 2.12.4 shutdown workaround')
    p.add_argument('--observe-only',action='store_true',help='start scene without autonomous motion')
    p.add_argument('--timeout',type=float,default=1800)
    p.add_argument('--domain',type=int,default=86)
    p.add_argument('--scenario',type=Path,help='Complete measured-scene configuration; legacy examples remain unchanged')
    p.add_argument('--replay-commands',type=Path,help='Replay saved joint commands through the same physical FSM; never replan')
    a=p.parse_args();a.order=a.order or ('A' if a.task=='basic' else 'CAB')
    if len(a.order)!=(1 if a.task=='basic' else 3) or len(set(a.order))!=len(a.order) or any(c not in 'ABCD' for c in a.order):p.error('invalid battery order')
    if a.output.exists() and any(a.output.iterdir()):p.error('output must be a new/empty directory')
    run=a.output.resolve();share=Path(get_package_share_directory('mtc_simulation'))
    prepare(share,run,a.task,a.scenario)
    scenario=json.loads((run/'scenario.json').read_text());scenario['requested_order']=a.order
    if a.replay_commands:scenario['replay_commands']=str(a.replay_commands.resolve(strict=True))
    (run/'scenario.json').write_text(json.dumps(scenario,indent=2)+'\n')
    env=os.environ.copy();env.update(ROS_DOMAIN_ID=str(a.domain),ROS_LOCALHOST_ONLY='1',
        GZ_PARTITION='mtc_v1_'+uuid.uuid4().hex,RCUTILS_COLORIZED_OUTPUT='0')
    env.setdefault('RMW_IMPLEMENTATION','rmw_fastrtps_cpp')
    env.setdefault('FASTDDS_BUILTIN_TRANSPORTS','UDPv4')
    env['GZ_SIM_SYSTEM_PLUGIN_PATH']=str(Path(get_package_prefix('mtc_simulation'))/'lib')+':'+env.get('GZ_SIM_SYSTEM_PLUGIN_PATH','')
    env['GZ_SIM_RESOURCE_PATH']=str(share.parent)+':'+env.get('GZ_SIM_RESOURCE_PATH','')
    processes=[];logs=[];progress=None
    def show_progress():
        nonlocal progress
        if progress is None:
            if not (run/'events.jsonl').exists():return
            progress=(run/'events.jsonl').open()
        while True:
            offset=progress.tell();line=progress.readline()
            if not line.endswith('\n'):progress.seek(offset);return
            event=json.loads(line)
            if event.get('event')=='state_entered':
                print(f"[{event['sim_time']:.1f}s] {event['object']}: {event['state']}",flush=True)
    def start(name,command):
        f=(run/(name+'.log')).open('w');logs.append(f)
        proc=subprocess.Popen(command,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append((name,proc));return proc
    def call(name,command,timeout=60):
        proc=start(name,command)
        if proc.wait(timeout=timeout):raise RuntimeError(name+' failed; see '+str(run/(name+'.log')))
    def alive():
        for name,proc in processes:
            if name in ('gazebo','state_publisher','bridge','move_group') and proc.poll() is not None:raise RuntimeError(name+' exited early')
    status={'success':False,'task':a.task,'order':a.order,'simulation_only':True,'physics':'passive contacts; no fixed attachment',
            'perception':'Gazebo ground truth','output':str(run),'domain':a.domain,'gz_partition':env['GZ_PARTITION'],
            'rmw':env['RMW_IMPLEMENTATION'],'fastdds_transports':env['FASTDDS_BUILTIN_TRANSPORTS']}
    exitcode=1
    try:
        # Refuse a domain that already has nodes. This process must own all motion endpoints.
        check=subprocess.run(['ros2','node','list','--no-daemon'],env=env,capture_output=True,text=True,timeout=20)
        if check.returncode or check.stdout.strip():raise RuntimeError('ROS domain is not empty; choose --domain')
        start('gazebo',['gz','sim','-v','3','-r']+([] if a.gui else ['-s','--headless-rendering'])+[str(run/'world.sdf')])
        params=run/'state_publisher.yaml'
        params.write_text(yaml.safe_dump({'/**':{'ros__parameters':{'robot_description':(run/'robot.urdf').read_text(),'use_sim_time':True}}}))
        start('state_publisher',['ros2','run','robot_state_publisher','robot_state_publisher','--ros-args','--params-file',str(params)])
        start('bridge',['ros2','run','ros_gz_bridge','parameter_bridge','/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                       '/overview/image@sensor_msgs/msg/Image[gz.msgs.Image'])
        start('camera',['ros2','run','mtc_simulation','record_camera.py',str(run)])
        call('spawn',['ros2','run','ros_gz_sim','create','-world','mtc','-name','mtc_s3','-file',str(run/'robot.urdf')])
        for name in ('joint_state_broadcaster','arm_controller'):
            call(name,['ros2','run','controller_manager','spawner',name,'--controller-manager-timeout','60'])
        start('move_group',['ros2','launch','mtc_simulation','v1_moveit.launch.py','run_dir:='+str(run),
                            'pin_plugins:='+str(a.pin_moveit_plugins).lower()])
        time.sleep(3);alive()
        (run/'processes.json').write_text(json.dumps({name:proc.pid for name,proc in processes},indent=2))
        if a.observe_only:
            print('Scene ready: '+str(run),flush=True)
            end=time.monotonic()+a.timeout
            while time.monotonic()<end:alive();time.sleep(.5)
            exitcode=0;status['observation_only']=True
        else:
            task=start('return_verify',[sys.executable,str(Path(__file__).with_name('verify_return_simulation.py')),
                '--candidate',str(Path(__file__).with_name('return_trajectory.json')),'--scene',str(run),
                '--math-dir',str(Path(__file__).resolve().parents[4]/'outputs/webui_laptop_20261014/hardware_integration/player_settle_v2'),
                '--domain',str(a.domain)])
            (run/'processes.json').write_text(json.dumps({name:proc.pid for name,proc in processes},indent=2))
            end=time.monotonic()+a.timeout
            while task.poll() is None:
                alive()
                if time.monotonic()>end:raise TimeoutError('Return verifier deadline')
                time.sleep(.2)
            status['task_exit_code']=task.returncode
            report=json.loads(Path(__file__).with_name('verification.json').read_text()) if Path(__file__).with_name('verification.json').exists() else {}
            status['success']=task.returncode==0 and report.get('passed',False)
            exitcode=0 if status['success'] else 1
    except (Exception,KeyboardInterrupt) as e:
        status['error']=str(e);print(str(e),file=sys.stderr)
    finally:
        # Only signal groups owned by this invocation; never kill another ROS/robot process.
        for name,proc in reversed(processes):
            if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
        shutdown_start=time.monotonic();deadline=shutdown_start+30
        retried_interrupt=False;sent_terminate=False;status['shutdown_escalation']=[]
        while any(proc.poll() is None for _,proc in processes) and time.monotonic()<deadline:
            elapsed=time.monotonic()-shutdown_start
            # Gazebo and its embedded ROS signal handlers can consume the first
            # interrupt without starting teardown. Retry only our process groups;
            # a nonzero exit or a final forced kill still fails the run.
            if elapsed>=3 and not retried_interrupt:
                for name,proc in reversed(processes):
                    if proc.poll() is None:
                        os.killpg(proc.pid,signal.SIGINT);status['shutdown_escalation'].append({'process':name,'signal':'SIGINT','elapsed_s':elapsed})
                retried_interrupt=True
            if elapsed>=12 and not sent_terminate:
                for name,proc in reversed(processes):
                    if proc.poll() is None:
                        os.killpg(proc.pid,signal.SIGTERM);status['shutdown_escalation'].append({'process':name,'signal':'SIGTERM','elapsed_s':elapsed})
                sent_terminate=True
            time.sleep(.1)
        for name,proc in reversed(processes):
            if proc.poll() is None:os.killpg(proc.pid,signal.SIGKILL)
            proc.wait(timeout=10)
        status['process_exit_codes']={name:proc.returncode for name,proc in processes}
        for f in logs:f.close()
        if progress is not None:progress.close()
        status['moveit_plugin_pinning']=a.pin_moveit_plugins
        status['child_failures']={name:re.findall(r'process has died[^\n]*', (run/(name+'.log')).read_text(errors='replace'))
                                 for name,_ in processes if 'process has died' in (run/(name+'.log')).read_text(errors='replace')}
        status['shutdown_clean']=not status['child_failures'] and all(proc.returncode==0 for _,proc in processes)
        if not status['shutdown_clean']:
            status['success']=False;exitcode=1
        (run/'run.json').write_text(json.dumps(status,indent=2)+'\n')
        print(json.dumps(status,ensure_ascii=False),flush=True)
    return exitcode

if __name__=='__main__':sys.exit(main())
