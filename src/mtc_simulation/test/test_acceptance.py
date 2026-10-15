"""An FSM success bit must not override independently observed physics failures."""
import importlib.util
import json
from pathlib import Path
import sys
import pytest

spec=importlib.util.spec_from_file_location('analyze_v1',Path(__file__).parents[1]/'scripts/analyze_v1.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

def recorded(tmp_path, *, lift=True, displaced=False, gap=False, collision=False):
    (tmp_path/'scenario.json').write_text(json.dumps({'task':'basic','targets':{'T0':[.3,-.1,.65]},'assumptions':{}}))
    (tmp_path/'result.json').write_text(json.dumps({'success':True,'order':'A','objects':[{'object':'battery_A','slot':'T0','success':True,'placement_verified':True}],
      'contacts':{'total':{'arm / table':{'kind':'unexpected','max_depth_m':.002}} if collision else {}}}))
    rows=[]
    for i in range(61):
        t=i*.1
        if gap and 44<i<50:continue
        xyz=[.3 if not displaced else .34,-.1,.65]
        if i<20 and lift:xyz[2]+=.03*i/20
        rows.append({'sim_time':t,'tcp':[0,0,0],'objects':{'battery_A':{'xyz':xyz,'quat':[0,0,0,1],'speed':0.0,'angular_speed':0.0}}})
    (tmp_path/'physics.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))

def test_measured_lift_and_stability(tmp_path,monkeypatch):
    monkeypatch.setitem(sys.modules,'matplotlib',None)
    recorded(tmp_path)
    assert module.analyze(tmp_path)['passed']

@pytest.mark.parametrize('fault',[{'lift':False},{'displaced':True},{'gap':True},{'collision':True}])
def test_success_bit_cannot_mask_physics_failure(tmp_path,monkeypatch,fault):
    monkeypatch.setitem(sys.modules,'matplotlib',None)
    recorded(tmp_path,**fault)
    assert not module.analyze(tmp_path)['passed']

@pytest.mark.parametrize('fault',['empty_order','wrong_slot','missing_angular_speed','tool_penetration'])
def test_incomplete_or_inconsistent_evidence_fails(tmp_path,monkeypatch,fault):
    monkeypatch.setitem(sys.modules,'matplotlib',None)
    recorded(tmp_path)
    p=tmp_path/'result.json';result=json.loads(p.read_text())
    if fault=='empty_order':result.update(order='',objects=[])
    if fault=='wrong_slot':result['objects'][0]['slot']='P3'
    if fault=='tool_penetration':result['contacts']['total']={'hook / bar':{'kind':'tool_contact','max_depth_m':.002}}
    if fault=='missing_angular_speed':
        q=tmp_path/'physics.jsonl';rows=[json.loads(s) for s in q.read_text().splitlines()]
        for r in rows:del r['objects']['battery_A']['angular_speed']
        q.write_text(''.join(json.dumps(r)+'\n' for r in rows))
    p.write_text(json.dumps(result))
    assert not module.analyze(tmp_path)['passed']

def test_center_tolerance_cannot_hide_footprint_outside_t0(tmp_path,monkeypatch):
    monkeypatch.setitem(sys.modules,'matplotlib',None)
    recorded(tmp_path)
    p=tmp_path/'scenario.json';scenario=json.loads(p.read_text())
    scenario.update(target_footprint_size_m=[.2,.1],target_yaw=0.)
    p.write_text(json.dumps(scenario))
    q=tmp_path/'physics.jsonl';rows=[json.loads(s) for s in q.read_text().splitlines()]
    for row in rows:row['objects']['battery_A']['xyz'][1]+=.012
    q.write_text(''.join(json.dumps(row)+'\n' for row in rows))
    report=module.analyze(tmp_path)
    assert report['objects'][0]['final_center_error_m']<.015
    assert not report['objects'][0]['footprint_inside_target']
    assert not report['passed']

def test_trial_lift_must_remain_airborne_for_required_duration(tmp_path,monkeypatch):
    monkeypatch.setitem(sys.modules,'matplotlib',None)
    recorded(tmp_path)
    p=tmp_path/'scenario.json';scenario=json.loads(p.read_text());scenario['lift_stability_s']=2.
    p.write_text(json.dumps(scenario))
    report=module.analyze(tmp_path)
    assert report['objects'][0]['max_lift_m']>.012
    assert not report['passed']

def recorded_sequence(tmp_path):
    scenario = dict(task='sequence', requested_order='ABC', assumptions={},
        initial_objects={c:[.3, .1+i*.1, .65] for i,c in enumerate('ABCD')},
        targets={'P1':[.3,-.1,.65], 'P2':[.3,-.25,.65], 'P3':[.15,-.25,.65]})
    result = dict(success=True, order='ABC', objects=[dict(object='battery_'+c, slot='P'+str(i+1),
        success=True, placement_verified=True) for i,c in enumerate('ABC')], contacts={'total':{}})
    rows=[]; events=[]; commands=[]
    for i,c in enumerate('ABC'):
        events.append(dict(event='selected', object='battery_'+c, sim_time=1+i*8))
        for j,p in enumerate(module.PHASES):
            commands.append(dict(object='battery_'+c, primitive=p))
            events.append(dict(event='execute', object='battery_'+c, primitive=p, sim_time=1.1+i*8+j*.1))
        events.append(dict(event='object_result', object='battery_'+c, sim_time=8+i*8, success=True, placement_verified=True))
    for tick in range(281):
        t=tick*.1; objects={}
        for i,c in enumerate('ABCD'):
            xyz=scenario['initial_objects'][c].copy()
            if c!='D':
                if t>=4+i*8: xyz=scenario['targets']['P'+str(i+1)].copy()
                elif t>=2+i*8: xyz[2]+=.03
            objects['battery_'+c]=dict(xyz=xyz, quat=[0,0,0,1], speed=0., angular_speed=0.)
        rows.append(dict(sim_time=t, tcp=[0,0,0], objects=objects))
    for name,data in [('scenario',scenario),('result',result)]:
        (tmp_path/(name+'.json')).write_text(json.dumps(data))
    for name,data in [('physics',rows),('events',events),('commands',commands)]:
        (tmp_path/(name+'.jsonl')).write_text(''.join(json.dumps(x)+'\n' for x in data))

def test_sequence_has_all_stages_and_undisturbed_placements(tmp_path,monkeypatch):
    monkeypatch.setitem(sys.modules,'matplotlib',None)
    recorded_sequence(tmp_path)
    report=module.analyze(tmp_path)
    assert report['passed']
    assert report['sequence_evidence']['executed_segments']==30

@pytest.mark.parametrize('fault',['missing_commands','wrong_object','missing_obstacle','disturbed_obstacle','disturbed_placement','early_pick'])
def test_sequence_success_cannot_mask_incomplete_or_disturbed_sequence(tmp_path,monkeypatch,fault):
    monkeypatch.setitem(sys.modules,'matplotlib',None)
    recorded_sequence(tmp_path)
    if fault=='missing_commands':
        (tmp_path/'commands.jsonl').unlink()
    elif fault=='wrong_object':
        p=tmp_path/'commands.jsonl';data=p.read_text();p.write_text(data.replace('battery_A','battery_B'))
    elif fault=='early_pick':
        p=tmp_path/'events.jsonl';events=[json.loads(s) for s in p.read_text().splitlines()]
        next(e for e in events if e['event']=='selected' and e['object']=='battery_B')['sim_time']=7.
        p.write_text(''.join(json.dumps(x)+'\n' for x in events))
    else:
        p=tmp_path/'physics.jsonl';rows=[json.loads(s) for s in p.read_text().splitlines()]
        if fault=='missing_obstacle':
            for r in rows:del r['objects']['battery_D']
        elif fault=='disturbed_obstacle': rows[150]['objects']['battery_D']['xyz'][0]+=.01
        else: rows[150]['objects']['battery_A']['xyz'][0]+=.03
        p.write_text(''.join(json.dumps(x)+'\n' for x in rows))
    assert not module.analyze(tmp_path)['passed']

@pytest.mark.parametrize('fault',[None,'missing','rotation','nan'])
def test_fixed_orientation_is_verified_along_the_entire_path(tmp_path,monkeypatch,fault):
    monkeypatch.setitem(sys.modules,'matplotlib',None)
    recorded(tmp_path)
    p=tmp_path/'scenario.json';cfg=json.loads(p.read_text());cfg['fixed_tcp_orientation_xyzw']=[0,0,0,1];p.write_text(json.dumps(cfg))
    p=tmp_path/'physics.jsonl';rows=[json.loads(s) for s in p.read_text().splitlines()]
    for row in rows:row['tcp_quat']=[0,0,0,1]
    if fault=='missing':del rows[10]['tcp_quat']
    if fault=='rotation':rows[10]['tcp_quat']=[0,0,.1,(1-.1**2)**.5]
    if fault=='nan':rows[10]['tcp_quat']=[float('nan'),0,0,1]
    p.write_text(''.join(json.dumps(x)+'\n' for x in rows))
    assert module.analyze(tmp_path)['passed'] == (fault is None)
