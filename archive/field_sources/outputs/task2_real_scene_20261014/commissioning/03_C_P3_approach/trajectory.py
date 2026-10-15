"""Portable trajectory math; standard library only, with radians and seconds."""
import bisect
import csv
import hashlib
import json
import math
from pathlib import Path

JOINTS=['shoulder_joint','upperArm_joint','foreArm_joint','wrist1_joint','wrist2_joint','wrist3_joint']
RAD=math.pi/180


def require(ok,message):
    if not ok:raise ValueError(message)


def vector(v,label='vector'):
    require(isinstance(v,list) and len(v)==6,label+' must contain six numbers')
    require(all(type(x) in (int,float) and math.isfinite(x) for x in v),label+' contains invalid numbers')
    return v


def interpolate(a,b,t):
    dt=b['t']-a['t'];require(dt>0,'Non-increasing waypoint times')
    u=max(0.,min(1.,(t-a['t'])/dt));q=[];v=[];acc=[]
    for i in range(6):
        c0=a['positions'][i];c1=a['velocities'][i]*dt;c2=a['accelerations'][i]*dt*dt/2
        d=b['positions'][i]-c0-c1-c2;e=b['velocities'][i]*dt-c1-2*c2;f=b['accelerations'][i]*dt*dt-2*c2
        c3=10*d-4*e+f/2;c4=-15*d+7*e-f;c5=6*d-3*e+f/2
        q.append(c0+u*(c1+u*(c2+u*(c3+u*(c4+u*c5)))))
        v.append((c1+u*(2*c2+u*(3*c3+u*(4*c4+u*5*c5))))/dt)
        acc.append((2*c2+u*(6*c3+u*(12*c4+u*20*c5)))/(dt*dt))
    return q,v,acc


def sample(segment,t):
    pts=segment['points'];times=[p['t'] for p in pts]
    if t<=times[0]:return tuple(pts[0][k].copy() for k in ('positions','velocities','accelerations'))
    if t>=times[-1]:return tuple(pts[-1][k].copy() for k in ('positions','velocities','accelerations'))
    i=bisect.bisect_right(times,t)-1
    return interpolate(pts[i],pts[i+1],t)


def dense(segment,interval=.01):
    duration=segment['points'][-1]['t'];n=max(2,math.ceil(duration/interval))
    dt=duration/n
    return dt,[sample(segment,i*dt) for i in range(n+1)]


def validate(bundle,limits=None):
    require(bundle['schema']==1 and bundle['joint_names']==JOINTS,'Wrong trajectory schema or joint order')
    require(bundle['units']=={'position':'rad','velocity':'rad/s','acceleration':'rad/s^2','time':'s'},'Wrong units')
    segments=bundle['segments'];require(bool(segments),'Empty trajectory')
    peaks_v=[0.]*6;peaks_a=[0.]*6;minimum=[math.inf]*6;maximum=[-math.inf]*6;duration=0.;seam=0.;previous=None
    for seg in segments:
        pts=seg['points'];require(len(pts)>=2 and pts[0]['t']==0,'Invalid segment start')
        prev=-1.
        for pt in pts:
            t=pt['t'];require(type(t) in (int,float) and math.isfinite(t) and t>prev,'Invalid waypoint time');prev=t
            for key in ('positions','velocities','accelerations'):vector(pt[key],key)
        if previous is not None:
            error=max(abs(x-y) for x,y in zip(previous,pts[0]['positions']));seam=max(seam,error)
            require(error<.01*RAD,'Discontinuous segment join')
        previous=pts[-1]['positions'];duration+=pts[-1]['t']
        _,samples=dense(seg)
        for q,v,a in samples:
            for j in range(6):
                require(all(math.isfinite(x) for x in (q[j],v[j],a[j])),'Nonfinite interpolation')
                peaks_v[j]=max(peaks_v[j],abs(v[j]));peaks_a[j]=max(peaks_a[j],abs(a[j]))
                minimum[j]=min(minimum[j],q[j]);maximum[j]=max(maximum[j],q[j])
                if limits:require(limits[0][j]+2*RAD<=q[j]<=limits[1][j]-2*RAD,f'J{j+1} violates two-degree limit margin')
    return dict(nominal_duration_s=duration,segments=len(segments),peak_velocity_deg_s=[v/RAD for v in peaks_v],
                peak_acceleration_deg_s2=[v/RAD for v in peaks_a],minimum_deg=[v/RAD for v in minimum],
                maximum_deg=[v/RAD for v in maximum],maximum_join_difference_deg=seam/RAD)


def load_package(root,kind):
    root=Path(root);manifest=json.loads((root/'manifest.json').read_text())
    require(kind in ('task','full'),'Unknown trajectory selection')
    filename={'task':'task_one_trajectory.json','full':'full_trajectory.json'}[kind]
    data=(root/filename).read_bytes()
    require(hashlib.sha256(data).hexdigest()==manifest['sha256'][filename],'Trajectory checksum mismatch')
    bundle=json.loads(data);validate(bundle,manifest['captured_limits'])
    require(bundle['plan_id']==manifest['plan_id'],'Plan identity mismatch')
    return bundle,manifest


def write_csv(path,bundle,interval=.02):
    offset=0.
    with Path(path).open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(['time_s','segment']+[f'J{i}_{unit}' for unit in ('rad','rad_s','rad_s2') for i in range(1,7)])
        for si,segment in enumerate(bundle['segments']):
            dt,points=dense(segment,interval)
            for i,(q,v,a) in enumerate(points):
                if si and i==0:continue
                writer.writerow([format(offset+i*dt,'.9f'),segment['primitive'],*q,*v,*a])
            offset+=segment['points'][-1]['t']

