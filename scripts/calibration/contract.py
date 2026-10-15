"""Offline TF snapshot checks. Never authorizes movement or fabricates missing TF."""
from collections import deque
from dataclasses import dataclass

from geometry import CalibrationError, Transform

# rclcpp::Time and the C++ core hold nanoseconds in int64; larger stamps cannot be exchanged.
INT64_MAX_NS = 2**63 - 1


def frame_name(value):
    if not isinstance(value, str) or not value or value.strip() != value or value.startswith('/') or any(c.isspace() for c in value):
        raise CalibrationError('INVALID_FRAME', 'frame must be nonempty, relative and contain no whitespace')
    return value


def positive_ns(value, label):
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise CalibrationError('INVALID_TIME', f'{label} must be positive integer ns; zero is not an observation timestamp')
    if value > INT64_MAX_NS:
        raise CalibrationError('INVALID_TIME', f'{label} exceeds int64 nanoseconds used by ROS 2 time')
    return value


@dataclass(frozen=True)
class Pose:
    frame_id: str
    stamp_ns: int
    time_domain: str
    transform: Transform

    @classmethod
    def from_dict(cls, d):
        if not isinstance(d, dict):
            raise CalibrationError('INVALID_POSE', 'pose must be an object')
        required = {'frame_id', 'stamp_ns', 'time_domain', 'length_unit', 'position',
                    'orientation_xyzw', 'valid', 'ring_pose_valid'}
        if not required.issubset(d):
            raise CalibrationError('INVALID_POSE', 'missing fields: ' + ','.join(sorted(required - d.keys())))
        if d['valid'] is not True or d['ring_pose_valid'] is not True:
            raise CalibrationError('INVALID_POSE', 'valid and ring_pose_valid must be boolean true')
        if d['length_unit'] != 'm':
            raise CalibrationError('UNIT_MISMATCH', 'position must be explicitly supplied in m')
        if d['time_domain'] not in ('ros', 'sim', 'synthetic'):
            raise CalibrationError('CLOCK_MISMATCH', 'unsupported observation time domain')
        return cls(frame_name(d['frame_id']), positive_ns(d['stamp_ns'], 'pose stamp'),
                   d['time_domain'], Transform(d['position'], d['orientation_xyzw']))


class FrameGraph:
    """A TF forest with exactly one parent per child. Dynamic inputs are snapshots."""
    def __init__(self, d):
        # type() rather than ==: True == 1 and 1.0 == 1 in Python, neither is schema 1.
        if not isinstance(d, dict) or type(d.get('schema_version')) is not int or d['schema_version'] != 1:
            raise CalibrationError('INVALID_CONFIG', 'expected schema_version 1')
        if d.get('length_unit') != 'm' or d.get('quaternion_order') != 'xyzw':
            raise CalibrationError('UNIT_MISMATCH', 'require m and quaternion order xyzw')
        self.data_kind = d.get('data_kind')
        self.time_domain = d.get('time_domain')
        if (self.data_kind not in ('REAL', 'SYNTHETIC') or
                self.time_domain not in ('ros', 'sim', 'synthetic') or
                (self.data_kind == 'REAL' and self.time_domain == 'synthetic')):
            raise CalibrationError('INVALID_CONFIG', 'invalid data_kind/time_domain combination')
        edges = d.get('transforms')
        if not isinstance(edges, list):
            raise CalibrationError('INVALID_CONFIG', 'transforms must be a list')
        self.edges = []
        self.parents = {}
        self.neighbors = {}
        for original in edges:
            if not isinstance(original, dict):
                raise CalibrationError('INVALID_CONFIG', 'transform entry must be an object')
            edge = dict(original)
            parent, child = frame_name(edge.get('parent')), frame_name(edge.get('child'))
            if parent == child or child in self.parents:
                raise CalibrationError('INVALID_TF_TREE', 'duplicate, self edge or multiple parents')
            if edge.get('status') not in ('MEASURED', 'ESTIMATED', 'UNKNOWN') or type(edge.get('static')) is not bool:
                raise CalibrationError('INVALID_CONFIG', 'each edge needs a valid status and boolean static')
            if edge['status'] == 'UNKNOWN':
                if edge.get('translation') is not None or edge.get('rotation_xyzw') is not None:
                    raise CalibrationError('UNKNOWN_HAS_NUMBERS', 'UNKNOWN transform must contain null geometry')
                edge['transform'] = None
            else:
                if edge['status'] == 'MEASURED' and (not isinstance(edge.get('evidence'), str) or not edge['evidence'].strip()):
                    raise CalibrationError('MISSING_EVIDENCE', 'MEASURED edge needs an evidence reference')
                edge['transform'] = Transform(edge.get('translation'), edge.get('rotation_xyzw'))
                if not edge['static']:
                    positive_ns(edge.get('stamp_ns'), 'dynamic snapshot stamp')
            self.parents[child] = parent
            index = len(self.edges)
            self.edges.append(edge)
            self.neighbors.setdefault(child, []).append((parent, index, False))
            self.neighbors.setdefault(parent, []).append((child, index, True))
        for child in self.parents:
            visited = set()
            current = child
            while current in self.parents:
                if current in visited:
                    raise CalibrationError('INVALID_TF_TREE', 'cycle in TF tree')
                visited.add(current)
                current = self.parents[current]

    def lookup(self, target, source, stamp_ns):
        target, source = frame_name(target), frame_name(source)
        if target not in self.neighbors or source not in self.neighbors:
            raise CalibrationError('MISSING_TF', f'{source} -> {target}: unknown frame')
        queue = deque([(source, [])])
        visited = {source}
        route = None
        while queue:
            current, path = queue.popleft()
            if current == target:
                route = path
                break
            for neighbor, index, invert in self.neighbors[current]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, path + [(index, invert)]))
        if route is None:
            raise CalibrationError('MISSING_TF', f'{source} -> {target}: disconnected tree')
        result = Transform.identity()
        for index, invert in route:
            edge = self.edges[index]
            if edge['status'] == 'UNKNOWN':
                raise CalibrationError('UNKNOWN_CALIBRATION', f"{edge['parent']} <- {edge['child']} is UNKNOWN")
            if self.data_kind == 'REAL' and edge['status'] != 'MEASURED':
                raise CalibrationError('UNMEASURED_CALIBRATION', 'REAL diagnostic requires measured edges with evidence')
            if not edge['static'] and edge['stamp_ns'] != stamp_ns:
                raise CalibrationError('TF_TIME_MISMATCH', 'dynamic snapshot not at observation time; no offline interpolation')
            t = edge['transform'].inverse() if invert else edge['transform']
            result = t.compose(result)
        return result


def transform_pose(graph, pose, target_frame, *, now_ns, max_age_ns, time_domain):
    positive_ns(now_ns, 'now')
    positive_ns(max_age_ns, 'max age')
    if time_domain != pose.time_domain or time_domain != graph.time_domain:
        raise CalibrationError('CLOCK_MISMATCH', 'pose, TF and now must use the same time domain')
    age = now_ns - pose.stamp_ns
    if age < 0:
        raise CalibrationError('FUTURE_POSE', 'observation is newer than now')
    if age > max_age_ns:
        raise CalibrationError('STALE_POSE', 'observation exceeds explicitly selected age threshold')
    t = graph.lookup(target_frame, pose.frame_id, pose.stamp_ns).compose(pose.transform)
    return {'status': 'DIAGNOSTIC_OK', 'frame_id': target_frame, 'stamp_ns': pose.stamp_ns,
            'time_domain': time_domain, 'length_unit': 'm', 'position': list(t.translation),
            'orientation_xyzw': list(t.rotation), 'age_ns': age,
            'valid': True, 'ring_pose_valid': True,
            'simulation_only': graph.data_kind == 'SYNTHETIC', 'motion_authorized': False,
            'calibration_status': 'SYNTHETIC_ONLY' if graph.data_kind == 'SYNTHETIC' else 'MEASURED_DECLARED'}
