"""Finish a pinned +45 degree mission at the user's requested 1 deg/s.

Read-only prepare by default. Absolute original endpoint survives an interrupted
attempt. At most two commands: 1 degree at 1 deg/s, then the remaining distance
at 1 deg/s. No retry, reversal, homing or mode change.
Software checks are not a safety-rated controller.
"""
import argparse
from collections import deque
from datetime import datetime, timezone
import getpass
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
import uuid

import j6_45 as base
import smoke
from telemetry import Telemetry

ROOT=Path(__file__).resolve().parent
RAD=base.RAD
require=smoke.require
MONITOR_PROFILE='rtde_1dps_window_150ms_v3'
MAIN_SPEED=1.0


def reference(config):
    spec=config['mission_45']
    data=(ROOT/spec['source_plan']).read_bytes()
    require(hashlib.sha256(data).hexdigest()==spec['sha256'], 'Original mission reference changed')
    p=json.loads(data)
    base.validate_identity(p['identity'],config)
    require(p['delta_deg']==45 and p['simulation'] is False, 'Wrong original mission')
    q=smoke.vector(p['initial_q'],'original start')
    final=smoke.vector(p['target_q'],'original endpoint')
    require(final[:5]==q[:5] and abs(final[5]-q[5]-45*RAD)<1e-12, 'Original target is not +45 degrees')
    return p


class Robot(base.Robot45):
    def __init__(self,config,user,password):
        self.stream=None
        super().__init__(config,user,password)
        try:self.stream=Telemetry(config['host'],user,password)
        except BaseException:self.close();raise

    def close(self):
        if self.stream is not None:self.stream.close();self.stream=None
        super().close()

    @staticmethod
    def convert(s):
        require(s['robot_mode']=='Running' and s['safety_mode'] in ('Normal','ReducedMode')
                and s['runtime']=='Stopped','Unsafe RTDE state')
        return {**s,'t':s['received'],'read_seconds':time.monotonic()-s['received']}

    def fast(self):
        return self.convert(self.stream.latest())

    def fast_samples(self):return [self.convert(s) for s in self.stream.drain()]

    def begin_stage(self):self.stream.drain()

    def move_stage(self,target,speed,acceleration):
        return self.motion.moveJoint(target,acceleration*RAD,speed*RAD,0.,0.)

    def stop(self):return self.motion.stopJoint(1*RAD)


def check_limits(robot, points):
    lo,hi=robot.limits()
    for q in points:
        for i,x in enumerate(smoke.vector(q,'point')):
            require(lo[i]+2*RAD<=x<=hi[i]-2*RAD,'Start/endpoint too close to joint limit')
    speeds=[smoke.vector(robot.config.getJointMaxSpeeds(),'hardware speeds'),
            smoke.vector(robot.config.getLimitJointMaxSpeeds(),'configured speeds')]
    accelerations=[smoke.vector(robot.config.getJointMaxAccelerations(),'hardware acceleration'),
                   smoke.vector(robot.config.getLimitJointMaxAccelerations(),'configured acceleration')]
    require(all(v[5]>=MAIN_SPEED*RAD for v in speeds+accelerations),'Configured limit does not permit requested profile')
    return dict(speed_limits_deg_s=[[math.degrees(x) for x in a] for a in speeds],
                acceleration_limits_deg_s2=[[math.degrees(x) for x in a] for a in accelerations])


def make_stages(start, final):
    require(final[5]-start[5]>.1*RAD,'Remaining distance too small for this staged profile')
    previous=start.copy();stages=[]
    for name,distance,speed,acceleration in [('probe_1_deg',1.,1.,1.),
                                             ('finish_original_45',45.,MAIN_SPEED,1.)]:
        if final[5]-previous[5]<=base.POSITION_TOL:break
        q=final.copy();q[5]=min(final[5],previous[5]+distance*RAD)
        stages.append(dict(name=name,target=q,speed_deg_s=speed,acceleration_deg_s2=acceleration))
        previous=q
    return stages


def stationary(robot, log, seconds=2., fraction=None, operation=None):
    s=base.stationary(robot,log,seconds=seconds,fraction=fraction,operation=operation)
    sample=robot.fast()
    require(sample['read_seconds']<=.1,'Fast feedback too slow')
    require(max(map(abs,sample['qd']))<=base.STILL_SPEED,'Fast feedback is not stationary')
    require(max(abs(a-b) for a,b in zip(sample['q'],s['q']))<=base.DRIFT_TOL,'Feedback channels disagree')
    log.event('stationary_fast',**sample)
    return s


def prepare(robot,config,log):
    p=reference(config)
    base.validate_identity(robot.identity(),config)
    base.validate_bare_flange(robot.config.getPayload(),robot.config.getTcpOffset())
    require(robot.runtime.isTaskAlive(0) is False,'A runtime task is active')
    s=stationary(robot,log)
    require(max(abs(a-b) for a,b in zip(s['q'][:5],p['initial_q'][:5]))<=base.DRIFT_TOL,
            'J1-J5 changed since original mission')
    require(p['initial_q'][5]-base.DRIFT_TOL<=s['q'][5]<p['target_q'][5], 'Outside original positive J6 sweep')
    stages=make_stages(s['q'],p['target_q'])
    limits=check_limits(robot,[s['q']]+[x['target'] for x in stages])
    result=dict(schema=2,monitor_profile=MONITOR_PROFILE,plan_id=uuid.uuid4().hex,created_utc=datetime.now(timezone.utc).isoformat(),
                host=config['host'],port=config['port'],identity=robot.identity(),
                mission_reference=config['mission_45'],initial_q=s['q'],original_q=p['initial_q'],
                final_q=p['target_q'],stages=stages,speed_fraction=s['speed_fraction'],
                operational_mode=s['operational_mode'],simulation=False,automatic_return=False,limits=limits)
    log.event('prepared_fast',plan=result)
    return result


def validate_plan(plan,robot,config,log):
    require(plan['schema']==2 and plan['mission_reference']==config['mission_45'],'Wrong mission')
    require(plan['monitor_profile']==MONITOR_PROFILE,'Monitoring profile changed; prepare again')
    require(plan['host']==config['host'] and plan['port']==config['port'],'Endpoint changed')
    require(len(plan['plan_id'])==32 and all(c in '0123456789abcdef' for c in plan['plan_id']),'Bad plan ID')
    age=(datetime.now(timezone.utc)-datetime.fromisoformat(plan['created_utc'])).total_seconds()
    require(0<=age<=600,'Plan expired')
    p=reference(config)
    require(plan['original_q']==p['initial_q'] and plan['final_q']==p['target_q'],'Mission endpoint changed')
    require(plan['stages']==make_stages(plan['initial_q'],plan['final_q']),'Stage parameters changed')
    require(plan['simulation'] is False and plan['automatic_return'] is False,'Invalid semantics')
    base.validate_identity(robot.identity(),config)
    require(plan['identity']==config['expected_identity'],'Plan identity changed')
    base.validate_bare_flange(robot.config.getPayload(),robot.config.getTcpOffset())
    require(robot.runtime.isTaskAlive(0) is False,'A runtime task is active')
    s=stationary(robot,log,fraction=plan['speed_fraction'],operation=plan['operational_mode'])
    require(max(abs(a-b) for a,b in zip(s['q'],plan['initial_q']))<=base.DRIFT_TOL,'Start moved after prepare')
    require(max(abs(a-b) for a,b in zip(s['q'][:5],plan['original_q'][:5]))<=base.DRIFT_TOL,'Other joints moved')
    check_limits(robot,[s['q']]+[x['target'] for x in plan['stages']])
    return s


class FeedbackGuard:
    """Check raw speed, position slope, target speed, and tracking error together."""
    def __init__(self,initial,target,original,speed):
        self.initial=initial;self.target=target;self.original=original;self.speed=speed
        self.window=deque();self.max_actual=0.;self.max_target=0.;self.max_slope=0.;self.max_tracking=0.

    def check(self,s):
        for key in ('q','qd','target_q','target_qd'):smoke.vector(s[key],key)
        require(math.isfinite(s['t']) and 0<=s['read_seconds']<=.1,'Fast feedback is slow/invalid')
        for i,q in enumerate(s['q']):
            lo,hi=sorted((self.initial[i],self.target[i]))
            require(lo-base.ENVELOPE_TOL<=q<=hi+base.ENVELOPE_TOL,f'J{i+1} escaped commanded segment')
            if i<5:
                require(abs(q-self.original[i])<=base.ENVELOPE_TOL,f'J{i+1} drift')
                require(abs(s['qd'][i])<=.4*RAD,f'Uncommanded J{i+1} speed')
        actual=abs(s['qd'][5])/RAD;target=abs(s['target_qd'][5])/RAD
        tracking=max(abs(a-b) for a,b in zip(s['q'],s['target_q']))/RAD
        self.max_actual=max(self.max_actual,actual);self.max_target=max(self.max_target,target)
        self.max_tracking=max(self.max_tracking,tracking)
        require(actual<=self.speed+.5,'Instantaneous speed above command + 0.5 deg/s')
        require(target<=self.speed+.05,'Controller target speed exceeds command')
        # Measured 1 deg/s probe: up to 0.038 deg of following offset. The
        # subsequent acceleration trace showed about 30-40 ms of target lead.
        # Use a bounded 50 ms travel allowance plus 0.02 deg, not a fixed
        # low-speed positional threshold. This is an engineering assumption,
        # NOT a vendor-certified following-error setting; factory settings stay
        # untouched. Raw speed, slope, other-joint drift and endpoint guards are
        # independent of this allowance. At 1 deg/s the limit is 0.07 deg.
        require(tracking<=.02+.05*self.speed,'Following offset exceeds 50 ms travel allowance + 0.02 deg')
        if self.window:
            require(0<s['t']-self.window[-1]['t']<=.2,'Feedback stopped advancing or has a gap')
        self.window.append(s)
        while len(self.window)>2 and s['t']-self.window[1]['t']>=.15:self.window.popleft()
        first=self.window[0];dt=s['t']-first['t']
        if dt>=.15:
            slope=abs(s['q'][5]-first['q'][5])/dt/RAD
            self.max_slope=max(self.max_slope,slope)
            require(slope<=self.speed+.2,'Position-derived speed exceeds command + 0.2 deg/s')
            # Compare the time-averaged difference over 150 ms. A single
            # quantized velocity sample is still subject to the hard peak cap;
            # it must not be confused with a sustained following failure.
            area=0.
            for a,b in zip(self.window,list(self.window)[1:]):
                ea=a['qd'][5]-a['target_qd'][5];eb=b['qd'][5]-b['target_qd'][5]
                area+=(ea+eb)*.5*(b['t']-a['t'])
            require(abs(area/dt)<=.75*RAD,'Sustained actual/target velocity discrepancy')


def segment(robot,plan,initial,stage,log):
    target=stage['target'];speed=stage['speed_deg_s'];acc=stage['acceleration_deg_s2']
    guard=FeedbackGuard(initial,target,plan['original_q'],speed)
    start=time.monotonic();deadline=start+max(30,20+2*(target[5]-initial[5])/RAD/(speed*plan['speed_fraction']))
    last_full=None;next_full=start;next_print=start;arrived_since=None;saw_motion=False
    robot.begin_stage()
    log.event('command_intent_fast',stage=stage,initial_q=initial)
    code=robot.move_stage(target,speed,acc)
    log.event('move_response_fast',stage=stage['name'],result=code)
    require(code==0,f'moveJoint returned {code}; no retry')
    while time.monotonic()<deadline:
        if time.monotonic()>=next_full:
            last_full=robot.snapshot()
            base.validate_state(last_full,fraction=plan['speed_fraction'],operation=plan['operational_mode'])
            require(last_full['read_seconds']<=.1,'Full status query too slow')
            log.event('status_fast',stage=stage['name'],**last_full)
            next_full=time.monotonic()+.1
        samples=robot.fast_samples()
        if not samples:time.sleep(.002);continue
        for s in samples:
            log.event('feedback_fast',stage=stage['name'],**s)
            guard.check(s)
        if abs(s['q'][5]-initial[5])>.05*RAD:saw_motion=True
        arrived=(saw_motion and max(abs(a-b) for a,b in zip(s['q'],target))<=base.POSITION_TOL
                 and max(map(abs,s['qd']))<=base.STILL_SPEED and last_full['exec_id']==-1 and last_full['queue']==0)
        if arrived:
            if arrived_since is None:arrived_since=time.monotonic()
            if time.monotonic()-arrived_since>=1:
                stats=dict(max_actual_deg_s=guard.max_actual,max_target_deg_s=guard.max_target,
                           max_position_slope_deg_s=guard.max_slope,max_tracking_deg=guard.max_tracking,
                           elapsed_seconds=time.monotonic()-start)
                log.event('stage_verified_fast',stage=stage['name'],q=s['q'],**stats)
                print('VERIFIED',stage['name'],json.dumps(stats),flush=True)
                return s
        else:arrived_since=None
        if time.monotonic()>=next_print:
            print(f'{stage["name"]}: J6={s["q"][5]/RAD:.4f}, target={target[5]/RAD:.4f}, '
                  f'actual speed={s["qd"][5]/RAD:.3f}, target speed={s["target_qd"][5]/RAD:.3f}',flush=True)
            next_print=time.monotonic()+2
        time.sleep(.01)
    raise smoke.Refused('Stage deadline reached; no retry')


def run_plan(robot,config,plan,log,ledger):
    require(config.get('allow_fast_motion') is True,'1 deg/s execution is locked')
    require(config.get('bare_flange_confirmed') is True and config.get('onsite_safety_confirmed') is True,
            'Reviewed bare-flange setup required')
    s=validate_plan(plan,robot,config,log)
    ledger.mkdir(exist_ok=True)
    with (ledger/(plan['plan_id']+'.attempted')).open('x') as f:
        f.write(str(log.path)+'\n');f.flush();os.fsync(f.fileno())
    try:
        for stage in plan['stages']:
            s=segment(robot,plan,s['q'],stage,log)
            s=stationary(robot,log,fraction=plan['speed_fraction'],operation=plan['operational_mode'])
            require(max(abs(a-b) for a,b in zip(s['q'],stage['target']))<=base.POSITION_TOL,'Stage hold error')
            base.validate_bare_flange(robot.config.getPayload(),robot.config.getTcpOffset())
        require(max(abs(a-b) for a,b in zip(s['q'],plan['final_q']))<=base.POSITION_TOL,'Original endpoint not reached')
    except BaseException:
        try:log.event('stop_attempt_fast',result=robot.stop())
        except BaseException:print('STOP DELIVERY UNKNOWN. Use onsite physical stop.',file=sys.stderr,flush=True)
        raise
    report=dict(success=True,physical_robot=True,simulation=False,plan_id=plan['plan_id'],
                original_q_deg=[x/RAD for x in plan['original_q']],final_q_deg=[x/RAD for x in s['q']],
                cumulative_delta_deg=[(b-a)/RAD for a,b in zip(plan['original_q'],s['q'])],
                commanded_main_speed_deg_s=MAIN_SPEED,automatic_return=False)
    log.event('mission_complete',**report)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=('prepare','execute'),nargs='?',default='prepare')
    parser.add_argument('--plan',type=Path)
    args=parser.parse_args();path=ROOT/'commission.local.json';config=smoke.load_config(path)
    if args.command=='execute':
        require(config.get('allow_fast_motion') is True,'1 deg/s execution is locked')
        require(args.plan is not None and sys.stdin.isatty(),'Execution needs an interactive terminal and a plan')
    username=input('SDK username [aubo]: ').strip() or 'aubo'
    password=getpass.getpass('SDK password (not saved): ')
    log=smoke.Log('fast45_'+args.command);robot=None
    with smoke.local_lock():
        try:
            robot=Robot(config,username,password)
            if args.command=='prepare':
                plan=prepare(robot,config,log)
                dest=ROOT/'plans'/('fast45-'+plan['plan_id']+'.json')
                dest.write_text(json.dumps(plan,indent=2))
                print('READ-ONLY PLAN:',dest)
                print(json.dumps(plan,indent=2))
            else:
                plan=json.loads(args.plan.read_text());validate_plan(plan,robot,config,log)
                print('Original endpoint J6:',plan['final_q'][5]/RAD,'deg. Stages:',
                      [(s['name'],s['target'][5]/RAD,s['speed_deg_s']) for s in plan['stages']],flush=True)
                require(input('Type FINISH ORIGINAL J6 +45 AT 1 DEG S: ').strip()==
                        'FINISH ORIGINAL J6 +45 AT 1 DEG S','Not confirmed')
                # Consume the persistent switch before any command; local config
                # retains authorization only for this one running process.
                require(smoke.load_config(path)==config,'Configuration changed during review')
                disarmed={**config,'allow_fast_motion':False}
                temp=path.with_suffix('.relock.tmp');temp.write_text(json.dumps(disarmed,indent=2))
                temp.chmod(0o600);os.replace(temp,path)
                result=run_plan(robot,config,plan,log,ROOT/'attempts')
                log.path.with_suffix('.summary.json').write_text(json.dumps(result,indent=2))
                print(json.dumps(result,indent=2))
        except BaseException as e:
            log.event('failed',error=type(e).__name__+': '+str(e));raise
        finally:
            if robot:robot.close()
            print('Log:',log.path)


if __name__=='__main__':
    def interrupt(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupt)
    try:main()
    except KeyboardInterrupt:print('Interrupted. Inspect physical stop state.',file=sys.stderr);sys.exit(130)
    except Exception as e:print('REFUSED / FAILED:',e,file=sys.stderr);sys.exit(1)
