"""Plot the accepted smooth reference; timing excludes controller checks."""
from pathlib import Path
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import BPoly

root=Path(__file__).resolve().parent
segments=[json.loads(s) for s in (root/'retiming_02/candidate_commands_input.jsonl').read_text().splitlines()]
report=json.loads((root/'retiming_02/retiming_validation.json').read_text())
times=[];curves=[[],[],[],[]];offset=0.;boundaries=[]
for index,segment in enumerate(segments):
    points=segment['points'];knots=np.array([p['t'] for p in points])
    data=np.array([[p[k] for k in ['positions','velocities','accelerations']] for p in points])
    poly=BPoly.from_derivatives(knots,data)
    probe=np.linspace(0,knots[-1],int(np.ceil(knots[-1]/.02))+1)
    times.extend(offset+probe)
    for derivative in range(4):curves[derivative].extend(np.rad2deg(poly(probe,nu=derivative)))
    offset+=knots[-1]
    if (index+1)%10==0:boundaries.append(offset)
fig,axes=plt.subplots(4,1,figsize=(12,9),sharex=True,constrained_layout=True)
labels=['Position (deg)','Velocity (deg/s)','Acceleration (deg/s^2)','Jerk (deg/s^3)']
limits=[None,report['maximum_velocity_deg_s'],report['maximum_acceleration_deg_s2'],report['maximum_jerk_deg_s3']]
for derivative,axis in enumerate(axes):
    axis.plot(times,np.asarray(curves[derivative]),linewidth=.9)
    axis.set_ylabel(labels[derivative]);axis.grid(alpha=.2)
    if limits[derivative] is not None:
        for sign in [-1,1]:axis.axhline(sign*limits[derivative],color='#ad3434',linewidth=.7,linestyle='--')
    for boundary in boundaries[:-1]:axis.axvline(boundary,color='#555',linestyle=':',linewidth=.9)
axes[0].legend(['J'+str(i) for i in range(1,7)],ncol=6,loc='upper right')
left=0.
for letter,right in zip('ABC',boundaries):
    axes[0].text((left+right)/2,1.02,letter+' -> P'+str('ABC'.index(letter)+1),
                 transform=axes[0].get_xaxis_transform(),ha='center')
    left=right
axes[-1].set_xlabel('Cumulative motion time (s); automatic checks and settling excluded')
fig.suptitle(f'Task 2: smoothed full sequence, {report["previous_motion_s"]:.2f} s -> {offset:.2f} s')
fig.savefig(root/'motion_profile.png',dpi=170)
print(root/'motion_profile.png')
