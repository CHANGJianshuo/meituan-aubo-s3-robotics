#!/usr/bin/env python3
"""Independently check recorded Gazebo poses against the requested task, and plot them."""
import argparse
import json
import math
from pathlib import Path

PHASES = ['side_insert', 'hook_engage', 'test_lift', 'lift', 'transport',
          'pre_seat', 'seat_contact', 'unload', 'side_withdraw', 'retreat']

def sequence_evidence(run, scenario, result, samples):
    """Require three distinct completed picks, with prior placements undisturbed."""
    try:
        events = [json.loads(line) for line in (run/'events.jsonl').read_text().splitlines() if line]
        commands = [json.loads(line) for line in (run/'commands.jsonl').read_text().splitlines() if line]
        order = scenario['requested_order']
        expected = [('battery_'+c, p) for c in order for p in PHASES]
        executed = [(e['object'], e['primitive']) for e in events if e['event'] == 'execute']
        recorded = [(c['object'], c['primitive']) for c in commands]
        selected = [e for e in events if e['event'] == 'selected']
        completed = [e for e in events if e['event'] == 'object_result']
        ordered = (executed == expected and recorded == expected
                   and [e['object'] for e in selected] == ['battery_'+c for c in order]
                   and [e['object'] for e in completed] == ['battery_'+c for c in order]
                   and all(e.get('success') and e.get('placement_verified') for e in completed)
                   and all(selected[i]['sim_time'] < completed[i]['sim_time'] for i in range(3))
                   and all(completed[i]['sim_time'] <= selected[i+1]['sim_time'] for i in range(2)))
        if not ordered:
            return dict(passed=False, reason='Missing or inconsistent chronological sequence execution evidence')
        moved_before_pick = {}
        disturbed_after_placement = {}
        for letter, initial in scenario['initial_objects'].items():
            key = 'battery_'+letter
            end = next((e['sim_time'] for e in selected if e['object'] == key), float('inf'))
            values = [s['objects'][key] for s in samples if s['sim_time'] <= end]
            if not values:
                return dict(passed=False, reason='Missing initial object observations: '+key)
            maximum = max(math.dist(o['xyz'], initial) for o in values)
            if maximum > .003:
                moved_before_pick[key] = maximum
        for i, event in enumerate(completed):
            key = event['object']; target = scenario['targets']['P'+str(i+1)]
            values = [s['objects'][key] for s in samples if s['sim_time'] >= event['sim_time']]
            if not values:
                return dict(passed=False, reason='No observation after placement: '+key)
            if any(math.hypot(o['xyz'][0]-target[0], o['xyz'][1]-target[1]) >= .015
                   or abs(o['xyz'][2]-target[2]) >= .002 or o['speed'] >= .003
                   or o['angular_speed'] >= .04 or 1-2*(o['quat'][0]**2+o['quat'][1]**2) <= .995
                   for o in values):
                disturbed_after_placement[key] = True
        return dict(passed=not moved_before_pick and not disturbed_after_placement,
                    executed_segments=len(executed), object_order=order,
                    moved_before_pick=moved_before_pick,
                    disturbed_after_placement=disturbed_after_placement)
    except (OSError, ValueError, KeyError, IndexError, TypeError) as error:
        return dict(passed=False, reason='Incomplete sequence evidence: '+str(error))

def fixed_orientation_evidence(scenario, samples):
    try:
        expected = scenario['fixed_tcp_orientation_xyzw']
        if len(expected) != 4 or not all(math.isfinite(x) for x in expected):
            raise ValueError('Invalid reference quaternion')
        if abs(sum(x*x for x in expected)-1.) > 1e-6:
            raise ValueError('Reference quaternion must be normalized')
        angles = []
        for sample in samples:
            q = sample['tcp_quat']
            if len(q) != 4 or not all(math.isfinite(x) for x in q) or abs(sum(x*x for x in q)-1.) > .001:
                raise ValueError('Invalid measured quaternion')
            dot = abs(sum(a*b for a,b in zip(q, expected)))/math.sqrt(sum(x*x for x in q))
            angles.append(math.degrees(2*math.acos(min(1., dot))))
        maximum = max(angles)
        return dict(passed=maximum <= .1, max_deviation_deg=maximum, tolerance_deg=.1,
                    samples=len(angles), source='Independent Gazebo TCP orientation trace')
    except (KeyError, ValueError, TypeError) as error:
        return dict(passed=False, reason='Incomplete fixed-orientation evidence: '+str(error))

def footprint_inside(observation, target, yaw, size):
    """Project all four battery footprint corners into the printed T0 rectangle."""
    qx,qy,qz,qw=observation['quat']
    xx,xy=1-2*(qy*qy+qz*qz),2*(qx*qy-qw*qz)
    yx,yy=2*(qx*qy+qw*qz),1-2*(qx*qx+qz*qz)
    c,s=math.cos(yaw),math.sin(yaw)
    for x in (-.035,.035):
        for y in (-.040,.040):
            dx=observation['xyz'][0]+xx*x+xy*y-target[0]
            dy=observation['xyz'][1]+yx*x+yy*y-target[1]
            if abs(c*dx+s*dy)>size[0]/2 or abs(-s*dx+c*dy)>size[1]/2:return False
    return True

def analyze(run):
    scenario=json.loads((run/'scenario.json').read_text())
    result=json.loads((run/'result.json').read_text())
    samples=[json.loads(line) for line in (run/'physics.jsonl').read_text().splitlines() if line.strip()]
    order=result.get('order','')
    count=1 if scenario['task']=='basic' else 3
    order_ok=(len(order)==count and len(set(order))==count and all(c in 'ABCD' for c in order)
              and order==scenario.get('requested_order',order)
              and [r.get('object') for r in result['objects']]==['battery_'+c for c in order])
    rows=[]
    for index,letter in enumerate(result['order']):
        key='battery_'+letter
        slot='T0' if scenario['task']=='basic' else 'P'+str(index+1)
        target=scenario['targets'][slot]
        data=[(s['sim_time'],s['objects'][key]) for s in samples if key in s['objects']]
        if not data:
            rows.append({'object':key,'passed':False,'reason':'no physics samples'});continue
        initial=data[0][1]['xyz'];final=data[-1][1]['xyz']
        target_size=scenario.get('target_footprint_size_m') if slot=='T0' else None
        footprint_ok=(target_size is None or footprint_inside(data[-1][1],target,scenario.get('target_yaw',0.),target_size))
        error=math.hypot(final[0]-target[0],final[1]-target[1])
        max_lift=max(o['xyz'][2]-initial[2] for _,o in data)
        start=data[-1][0];last=start
        for t,o in reversed(data):
            x,y,z=o['xyz'];qx,qy,qz,qw=o['quat']
            upright=1-2*(qx*qx+qy*qy)
            if last-t>.3 or math.hypot(x-target[0],y-target[1])>=.015 or abs(z-target[2])>=.002 or o['speed']>=.003 or o.get('angular_speed',float('inf'))>=.04 or upright<=.995:break
            if target_size is not None and not footprint_inside(o,target,scenario.get('target_yaw',0.),target_size):break
            start=t;last=t
        stable=data[-1][0]-start
        fsm=next((r for r in result['objects'] if r['object']==key),{})
        airborne_start=None;airborne_stable=0.;previous=None
        for t,o in data:
            good=o['xyz'][2]-initial[2]>.012 and o['speed']<.003 and o.get('angular_speed',float('inf'))<.04
            if not good or (previous is not None and t-previous>.3):airborne_start=None
            if good:
                if airborne_start is None:airborne_start=t
                airborne_stable=max(airborne_stable,t-airborne_start)
            previous=t
        lift_ok=airborne_stable>=scenario.get('lift_stability_s',0.)-1e-6
        passed=error<.015 and max_lift>.012 and stable>=3.0-1e-6 and footprint_ok and lift_ok and fsm.get('placement_verified',False)
        passed=passed and fsm.get('slot')==slot and fsm.get('success',False)
        rows.append({'object':key,'slot':slot,'passed':passed,'max_lift_m':max_lift,
                     'final_center_error_m':error,'final_xyz':final,'stable_from_trace_s':stable,
                     'footprint_inside_target':footprint_ok,'airborne_stable_from_trace_s':airborne_stable})
    pairs=result.get('contacts',{}).get('total',{})
    unexpected={k:v for k,v in pairs.items() if v['kind']=='unexpected' and v['max_depth_m']>.001}
    penetrating_tool={k:v for k,v in pairs.items() if v['kind']=='tool_contact' and v['max_depth_m']>.001}
    sequence = sequence_evidence(run, scenario, result, samples) if scenario['task']=='sequence' else None
    report={'passed':bool(result['success']) and order_ok and bool(rows) and all(r['passed'] for r in rows) and not unexpected and not penetrating_tool,
            'requested_order_verified':order_ok,'tool_contacts_over_1mm':penetrating_tool,
            'objects':rows,'unexpected_contacts_over_1mm':unexpected,
            'acceptance':'15 mm center tolerance, 2 mm base height tolerance, upright, >=3 s stable; not competition scoring',
            'inner_circle_footprint_fits':math.hypot(.07,.08)<=.08,
            'model_limits':scenario['assumptions']}
    if sequence is not None:
        report['sequence_evidence'] = sequence
        report['passed'] = report['passed'] and sequence['passed']
    if 'fixed_tcp_orientation_xyzw' in scenario:
        orientation = fixed_orientation_evidence(scenario, samples)
        report['fixed_orientation'] = orientation
        report['passed'] = report['passed'] and orientation['passed']
    (run/'acceptance.json').write_text(json.dumps(report,indent=2)+'\n')
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(1,2,figsize=(12,5),layout='constrained')
        for letter,color in zip('ABCD',['#cc3333','#c49a00','#3355bb','#228844']):
            key='battery_'+letter
            data=[(s['sim_time'],s['objects'][key]['xyz']) for s in samples if key in s['objects']]
            if not data:continue
            axes[0].plot([x[1][0] for x in data],[x[1][1] for x in data],color=color,label=letter)
            axes[0].scatter(*data[-1][1][:2],color=color,s=35)
            axes[1].plot([x[0]-samples[0]['sim_time'] for x in data],[(x[1][2]-scenario.get('table_z',.65))*1000 for x in data],color=color,label=letter)
        for slot,v in scenario['targets'].items():
            axes[0].scatter(v[0],v[1],marker='x',color='black');axes[0].annotate(slot,v[:2])
        axes[0].set(xlabel='World X (m)',ylabel='World Y (m)',title='Measured battery paths',aspect='equal')
        axes[1].set(xlabel='Simulation time (s)',ylabel='Battery base above table (mm)',title='Physical lift and placement')
        for ax in axes:ax.grid(alpha=.2);ax.legend()
        fig.savefig(run/'trajectories.png',dpi=160);plt.close(fig)
    except ImportError:
        pass # Acceptance never depends on optional plotting.
    return report

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('run',type=Path);args=parser.parse_args()
    result=analyze(args.run);print(json.dumps(result,indent=2));raise SystemExit(0 if result['passed'] else 1)
