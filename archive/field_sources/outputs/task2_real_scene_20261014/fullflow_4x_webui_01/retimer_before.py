#!/usr/bin/env python3
"""Retime a proven joint path, with zero velocity/acceleration at every stop.

Offline only. Smooths the original waypoint paths within a checked geometric
tolerance, including the intermediate overhead stop. Complete simulation is
required before hardware use.
"""
import argparse
import copy
from datetime import datetime
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.interpolate import BPoly, make_interp_spline
from scipy.optimize import brentq


def warp(u, width):
    """Monotone clock with quintic velocity ramps and a constant middle rate."""
    u = np.asarray(u)
    left = u <= .5
    z = np.minimum(u, 1-u)/width
    ramp = z < 1
    z = np.minimum(z, 1)
    gain = 1/(1-width)
    distance = np.where(ramp, width*(z**6-3*z**5+2.5*z**4),
                        np.minimum(u, 1-u)-width/2)*gain
    rate = np.where(ramp, 6*z**5-15*z**4+10*z**3, 1)*gain
    accel = np.where(ramp, 30*z**4-60*z**3+30*z**2, 0)*gain/width
    jerk = np.where(ramp, 120*z**3-180*z**2+60*z, 0)*gain/width**2
    return np.where(left, distance, 1-distance), rate, np.where(left, accel, -accel), jerk


def evaluate(poly, start, end, u, duration, width):
    g, d, dd, ddd = warp(u, width)
    span = end-start
    tau = start+span*g
    v = span*d/duration
    a = span*dd/duration**2
    j = span*ddd/duration**3
    p0, p1, p2, p3 = [poly(tau, nu=n) for n in range(4)]
    return (p0, p1*v[:, None],
            p2*v[:, None]**2+p1*a[:, None],
            p3*v[:, None]**3+3*p2*v[:, None]*a[:, None]+p1*j[:, None])


def bounded_spline(poly, knots, model, forward):
    """Refine original polynomial samples until C4 fitting preserves geometry."""
    a,b = knots[0],knots[-1]
    ts = np.linspace(a,b,12001)
    oldq = poly(ts)
    oldf = forward(model,oldq)
    tool_points = [np.array([0.,.045,.1105]),np.array([0.,.075,.1105])]
    old_tool = [oldf[:,:3,3]+np.einsum('nij,j->ni',oldf[:,:3,:3],point) for point in tool_points]
    for subdivisions in [1,2,4,8,16]:
        refined = np.r_[np.concatenate([np.linspace(lo,hi,subdivisions+1)[:-1]
                                       for lo,hi in zip(knots,knots[1:])]),b]
        smooth = make_interp_spline(refined,poly(refined),k=5,
            bc_type=([(1,poly(a,nu=1)),(2,poly(a,nu=2))],
                     [(1,poly(b,nu=1)),(2,poly(b,nu=2))]))
        newq = smooth(ts)
        joint_deviation = float(np.rad2deg(np.abs(oldq-newq).max()))
        newf = forward(model,newq)
        tool_deviation = max(float(1000*np.linalg.norm(
            previous-(newf[:,:3,3]+np.einsum('nij,j->ni',newf[:,:3,:3],point)),axis=1).max())
            for point,previous in zip(tool_points,old_tool))
        if joint_deviation < .02 and tool_deviation < .1:
            return smooth,joint_deviation,tool_deviation,subdivisions
    raise ValueError(f'Smoothing exceeds path bounds: {joint_deviation:.6f}deg, {tool_deviation:.6f}mm')


def sampling_clock(knots, start, end, duration, width):
    seed = [brentq(lambda u: float(warp(np.array(u),width)[0])-float((t-start)/(end-start)),
                   0,1) for t in knots[1:-1]]
    candidates=np.unique(np.r_[np.linspace(0,1,math.ceil(duration/.02)+1),seed,width,1-width])
    retained=[0.]
    # Microsecond intervals amplify floating-point errors in reconstructed
    # derivatives. A 1 ms gap keeps the exported polynomial well conditioned;
    # its deviation from the analytic path is independently checked below.
    for x in candidates[1:-1]:
        if (x-retained[-1])*duration >= .001 and (1-x)*duration >= .001:
            retained.append(float(x))
    return np.array(retained+[1.])


def retime(args):
    if not math.isfinite(args.ramp_fraction) or not 0 < args.ramp_fraction < .5:
        raise ValueError('Ramp fraction must be finite and between zero and one half')
    if any(not math.isfinite(x) or x <= 0 for x in
           [args.velocity_deg_s,args.acceleration_deg_s2,args.jerk_deg_s3]):
        raise ValueError('Velocity, acceleration and jerk bounds must be finite and positive')
    original = json.loads(args.source.read_text())
    bundle = copy.deepcopy(original)
    state = json.loads(args.state.read_text())
    old_motion = sum(s['points'][-1]['t'] for s in original['segments'])
    motion_ratio = getattr(args, 'motion_ratio', None)
    if motion_ratio is None:
        if args.previous_log is None:
            raise ValueError('A previous successful task-one log or --motion-ratio is required')
        rows = [json.loads(s) for s in args.previous_log.read_text().splitlines() if s]
        moves = [r for r in rows if r['event'] == 'move_intent']
        completions = [r for r in rows if r['event'] == 'completed']
        assert len(moves) == len(original['segments']) == 10 and len(completions) == 1
        assert not any(r['event'] == 'failed' for r in rows)
        observed = (datetime.fromisoformat(completions[0]['utc'])-
                    datetime.fromisoformat(moves[0]['utc'])).total_seconds()
        overhead = observed-old_motion
        desired_motion = observed*args.total_ratio-overhead
    else:
        if not math.isfinite(motion_ratio) or not 0 < motion_ratio <= 1:
            raise ValueError('Motion ratio must be finite and in (0, 1]')
        phases = ['side_insert','hook_engage','test_lift','lift','transport',
                  'pre_seat','seat_contact','unload','side_withdraw','retreat']
        order = original.get('order', '')
        if len(order) not in (1,3) or len(set(order)) != len(order) or any(c not in 'ABCD' for c in order):
            raise ValueError('A complete task order is required for motion-ratio retiming')
        expected = [('battery_'+letter,phase) for letter in order for phase in phases]
        if [(s.get('object'),s['primitive']) for s in original['segments']] != expected:
            raise ValueError('Incomplete or reordered task trajectory')
        observed, overhead = None, 0.
        desired_motion = old_motion*motion_ratio
    assert desired_motion > 0
    limits = np.deg2rad([args.velocity_deg_s, args.acceleration_deg_s2, args.jerk_deg_s3])
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from fit_kinematics import origins, forward
    model = origins(ET.parse(args.urdf).getroot())
    floors = {'hook_engage': 2.8, 'test_lift': 1.8, 'seat_contact': 1.5,
              'unload': 1.0, 'side_withdraw': 2.6}
    parts = []
    joins = []
    previous = None
    for si, segment in enumerate(bundle['segments']):
        points = segment['points']
        if previous is not None:
            correction = np.array(previous)-points[0]['positions']
            assert np.rad2deg(np.abs(correction).max()) < .001
            joins.append({'stage': si+1, 'start_correction_deg': np.rad2deg(correction).tolist()})
            points[0]['positions'] = list(previous)
        previous = points[-1]['positions']
        knots = np.array([p['t'] for p in points])
        derivatives = np.array([[p[k] for k in ('positions', 'velocities', 'accelerations')] for p in points])
        poly = BPoly.from_derivatives(knots, derivatives)
        stops = sorted(set([0, len(points)-1]+[i for i, p in enumerate(points)
                       if np.linalg.norm(p['velocities']) < 1e-6]))
        for lo, hi in zip(stops, stops[1:]):
            a, b = knots[lo], knots[hi]
            # C4 quintic spline through the same measured-path waypoints. Keep
            # the original boundary derivatives; the clock separately makes
            # physical start/stop derivatives zero. This removes interior jerk
            # jumps without imposing an artificial curvature at short end knots.
            smooth,joint_deviation,tool_deviation,refinement = bounded_spline(poly,knots[lo:hi+1],model,forward)
            u = np.linspace(0, 1, 12001)
            _, v, acc, jerk = evaluate(smooth, a, b, u, 1, args.ramp_fraction)
            minimum = max(np.abs(v).max()/limits[0],
                          math.sqrt(np.abs(acc).max()/limits[1]),
                          np.cbrt(np.abs(jerk).max()/limits[2]),
                          floors.get(segment['primitive'], 0) if len(stops) == 2 else 0)
            parts.append(dict(stage=si, primitive=segment['primitive'], start=float(a), end=float(b),
                              minimum=float(minimum), poly=smooth, knots=knots[lo:hi+1],
                              joint_deviation=joint_deviation, tool_deviation=tool_deviation,
                              spline_refinement=refinement))
    minimum_motion = sum(p['minimum'] for p in parts)
    # Reserve a numerical margin below the chosen kinematic caps. If these
    # constraints need a little more time, report that instead of weakening them.
    if motion_ratio is not None and minimum_motion*1.01 > desired_motion:
        raise ValueError(f'Requested motion time {desired_motion:.6f}s is below the bounded minimum '
                         f'{minimum_motion*1.01:.6f}s; constraints were not relaxed')
    new_motion = max(desired_motion, minimum_motion*1.01)
    factor = new_motion/minimum_motion
    validation = []
    grouped = [[] for _ in bundle['segments']]
    for part in parts:
        duration = part['minimum']*factor
        a, b, poly = part['start'], part['end'], part['poly']
        u = sampling_clock(part['knots'],a,b,duration,args.ramp_fraction)
        q, v, acc, _ = evaluate(poly, a, b, u, duration, args.ramp_fraction)
        v[[0, -1]] = 0
        acc[[0, -1]] = 0
        dst = grouped[part['stage']]
        offset = dst[-1]['t'] if dst else 0.
        for i, x in enumerate(u):
            if dst and i == 0:
                assert np.max(np.abs(np.array(dst[-1]['positions'])-q[i])) < 1e-12
                continue
            dst.append(dict(t=float(offset+x*duration), positions=q[i].tolist(),
                            velocities=v[i].tolist(), accelerations=acc[i].tolist()))
        # Check the exported quintic interpolation, not only the analytic clock.
        test_u = np.linspace(0, 1, math.ceil(duration/.001)+1)
        exact = evaluate(poly, a, b, test_u, duration, args.ramp_fraction)
        reconstructed = BPoly.from_derivatives(u*duration, np.stack([q, v, acc], axis=1))
        peaks = [float(np.rad2deg(np.abs(reconstructed(test_u*duration, nu=i)).max())) for i in (1, 2, 3)]
        # Bernstein polynomials lie in the convex hull of their coefficients.
        # These conservative bounds cover EVERY interval, including intervals
        # shorter than the sampling grid, so numerical jerk spikes cannot hide.
        bounds = [float(np.rad2deg(np.abs(reconstructed.derivative(i).c).max())) for i in (1,2,3)]
        deviation = float(np.rad2deg(np.abs(reconstructed(test_u*duration)-exact[0]).max()))
        assert deviation < .00001
        assert peaks[0] <= args.velocity_deg_s and peaks[1] <= args.acceleration_deg_s2
        assert peaks[2] <= args.jerk_deg_s3
        assert all(value <= limit for value,limit in zip(bounds,
                   [args.velocity_deg_s,args.acceleration_deg_s2,args.jerk_deg_s3]))
        validation.append(dict(stage=part['stage']+1, object=bundle['segments'][part['stage']].get('object'), primitive=part['primitive'],
            source_time_interval_s=[a, b], duration_s=duration,
            peak_velocity_deg_s=peaks[0], peak_acceleration_deg_s2=peaks[1],
            peak_jerk_deg_s3=peaks[2], maximum_interpolation_deviation_deg=deviation))
        validation[-1].update(spline_path_max_joint_deviation_deg=part['joint_deviation'],
                              spline_path_max_tool_deviation_mm=part['tool_deviation'],
                              spline_refinement=part['spline_refinement'],
                              bernstein_derivative_bounds_deg=bounds,
                              minimum_export_interval_s=float(np.diff(u*duration).min()))
    for si, segment in enumerate(bundle['segments']):
        segment['points'] = grouped[si]
        ns = np.array([int(p['t']*1e9) for p in segment['points']], dtype=np.int64)
        assert np.all(np.diff(ns) > 0)
        assert np.max(np.abs(np.array(segment['points'][-1]['positions'])-
                            original['segments'][si]['points'][-1]['positions'])) < 1e-12
        for p in [segment['points'][0], segment['points'][-1]]:
            assert p['velocities'] == [0.]*6 and p['accelerations'] == [0.]*6
    spec = importlib.util.spec_from_file_location('trajectory_math', args.math)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    stats = module.validate(bundle, state['limits'])
    cfg = state['robot_configuration']
    for i in range(6):
        assert np.deg2rad(stats['peak_velocity_deg_s'][i]) < min(cfg['getJointMaxSpeeds'][i], cfg['getLimitJointMaxSpeeds'][i])
        assert np.deg2rad(stats['peak_acceleration_deg_s2'][i]) < min(cfg['getJointMaxAccelerations'][i], cfg['getLimitJointMaxAccelerations'][i])
    if args.output.exists() and any(args.output.iterdir()):
        raise ValueError('Use a new/empty retiming output directory')
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'candidate_commands_input.jsonl').write_text(''.join(json.dumps(s)+'\n' for s in bundle['segments']))
    report = dict(offline_only=True, simulation_replay_pending=True, source=str(args.source.resolve()),
        source_sha256=hashlib.sha256(args.source.read_bytes()).hexdigest(),
        command_input_sha256=hashlib.sha256((args.output/'candidate_commands_input.jsonl').read_bytes()).hexdigest(),
        implementation_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        previous_log=str(args.previous_log.resolve()) if args.previous_log is not None else None, measured_previous_full_duration_s=observed,
        previous_motion_s=old_motion, estimated_preserved_checks_and_settling_s=overhead,
        requested_total_ratio=args.total_ratio if motion_ratio is None else None,
        target_full_duration_s=observed*args.total_ratio if observed is not None else None,
        requested_motion_ratio=motion_ratio, actual_motion_ratio=new_motion/old_motion,
        planned_motion_s=new_motion, expected_full_duration_s=new_motion+overhead if observed is not None else None,
        estimated_total_ratio=(new_motion+overhead)/observed if observed is not None else None,
        maximum_velocity_deg_s=args.velocity_deg_s, maximum_acceleration_deg_s2=args.acceleration_deg_s2,
        maximum_jerk_deg_s3=args.jerk_deg_s3, ramp_fraction=args.ramp_fraction,
        method='C4 quintic spline through original path waypoints, bounded geometric deviation; monotone clock with quintic rate ramps; original interior stops and exact endpoints preserved; C2 rest-to-rest joins',
        all_controller_and_tracking_guards_unchanged=True, join_corrections=joins,
        part_validation=validation, statistics=stats,
        timing_note=('Observed first move to completed; contains existing automatic checks. Wall-time prediction is not a hard real-time guarantee.'
                     if observed is not None else 'Motion duration only; automatic checks, contact dwell and settling are preserved and excluded from the requested ratio.'))
    (args.output/'retiming_validation.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k: report[k] for k in ['planned_motion_s', 'expected_full_duration_s', 'estimated_total_ratio', 'statistics']}, indent=2))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--previous-log', type=Path)
    p.add_argument('--state', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--math', type=Path, required=True)
    p.add_argument('--urdf', type=Path, required=True)
    p.add_argument('--total-ratio', type=float, default=.5)
    p.add_argument('--motion-ratio', type=float, help='Retime all task motion to this ratio; preserve automatic checks separately')
    p.add_argument('--velocity-deg-s', type=float, default=13.)
    p.add_argument('--acceleration-deg-s2', type=float, default=25.)
    p.add_argument('--jerk-deg-s3', type=float, default=200.)
    p.add_argument('--ramp-fraction', type=float, default=.08)
    retime(p.parse_args())
