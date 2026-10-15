"""Task descriptions derived from the prepared hardware trajectory."""
PHASES = ['side_insert', 'hook_engage', 'test_lift', 'lift', 'transport',
          'pre_seat', 'seat_contact', 'unload', 'side_withdraw', 'retreat']
PHASE_LABELS = ['接近把手', '水平插入', '试提起', '垂直提起', '搬运',
                '预放置', '接触支撑面', '卸载', '退出把手', '撤离']
COLORS = {'A': (1, '红'), 'B': (3, '黄'), 'C': (2, '蓝'), 'D': (4, '绿')}


def tracking_limits(manifest):
    """Accept only exact prepared policies with an unchanged request record."""
    if (manifest.get('uniform_tracking_guard_deg') == .5
            and 'tracking_guard_deg' not in manifest and 'tracking_guard_policy' not in manifest):
        return [.5]*6
    if (manifest.get('tracking_guard_policy') == 'task2_tracking_monitor_v1'
            and 'tracking_guard_deg' in manifest and manifest['tracking_guard_deg'] is None
            and manifest.get('tracking_guard_action') == 'record_only'
            and manifest.get('uniform_tracking_guard_deg') is None):
        import hashlib
        import json
        from pathlib import Path
        request=manifest.get('tracking_guard_request') or {}
        try:
            raw=Path(request['path']).read_bytes()
            record=json.loads(raw)
            if (hashlib.sha256(raw).hexdigest()==request['sha256']
                    and 'tracking_guard_deg' in record and record['tracking_guard_deg'] is None
                    and record.get('tracking_guard_action')=='record_only'
                    and record.get('user_submission_only') is True):
                return None
        except (KeyError,OSError,TypeError,ValueError):
            pass
        raise ValueError('Monitor-only tracking needs its exact pinned user request')
    policies = {'task2_j4_0p6_v1': [.5,.5,.5,.6,.5,.5],
                'task2_all_0p6_v1': [.6]*6}
    limits = policies.get(manifest.get('tracking_guard_policy'))
    if (manifest.get('uniform_tracking_guard_deg') is None
            and limits is not None
            and manifest.get('tracking_guard_deg') == limits):
        import hashlib
        from pathlib import Path
        request = manifest.get('tracking_guard_request') or {}
        try:
            if hashlib.sha256(Path(request['path']).read_bytes()).hexdigest() == request['sha256']:
                return limits
        except (KeyError, OSError, TypeError):
            pass
    raise ValueError('Task two tracking policy is unsupported or its request record changed')


def color_masks(image, roi, sequence=False):
    """Return red, yellow, green, blue masks (not the UI color-enum order).

    Task two uses the HSV calibration from verify_scene_unchanged.py, shared
    by both the starting-scene and final-placement checks. Its lime green
    straddles the legacy task-one hue boundary and includes low-saturation
    surfaces. Geometry, overlap and stability tolerances remain independent.
    """
    import cv2
    x0, y0, x1, y1 = roi
    h, s, v = cv2.split(cv2.cvtColor(image[y0:y1, x0:x1], cv2.COLOR_BGR2HSV))
    if sequence:
        return [((h <= 10) | (h > 170)) & (s > 95) & (v > 65),
                (h >= 16) & (h <= 38) & (s > 95) & (v > 65),
                (h >= 39) & (h <= 84) & (s > 50) & (v > 65),
                (h >= 85) & (h <= 130) & (s > 95) & (v > 65)]
    common = (s > 100) & (v > 55)
    return [common & ((h < 10) | (h > 170)),
            common & (h >= 15) & (h <= 40),
            common & (h > 40) & (h <= 80),
            common & (h >= 85) & (h <= 120)]


def describe(bundle, stage_offset=0):
    if bundle.get('task') == 'sequence':
        order = bundle.get('order')
        assignments = [{'object': 'battery_'+c, 'slot': 'P'+str(i+1)}
                       for i, c in enumerate('ABC')]
        if (order != 'ABC' or stage_offset != 0 or bundle.get('assignments') != assignments
                or [(s.get('object'), s.get('primitive')) for s in bundle['segments']] !=
                   [('battery_'+c, phase) for c in order for phase in PHASES]):
            raise ValueError('Task two requires the complete prepared ABC thirty-stage trajectory')
        return dict(mode=2, ordered_colors=[1, 3, 2], assignments=assignments,
                    title='红 → P1，黄 → P2，蓝 → P3', total_stages=30)
    return dict(mode=1, ordered_colors=[1],
                assignments=[dict(object='battery_A', slot='T0')],
                title='红电池 → T0', total_stages=10)


def stage_description(profile, segment, index, offset=0):
    absolute = index+offset
    object_index = (absolute-1)//10 if profile['mode'] == 2 else 0
    assignment = profile['assignments'][object_index]
    if segment.get('object') != assignment['object']:
        raise ValueError('Motion stage does not match its prepared object')
    letter = assignment['object'][-1]
    color, label = COLORS[letter]
    phase = PHASE_LABELS[PHASES.index(segment['primitive'])]
    return dict(current_index=object_index, current_slot=assignment['slot'],
                current_object_id=assignment['object'], current_color=color,
                detail=f'实机第 {absolute}/{profile["total_stages"]} 段：{label}块 → {assignment["slot"]} · {phase}')
