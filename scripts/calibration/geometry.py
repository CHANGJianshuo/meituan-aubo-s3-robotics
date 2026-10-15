"""SE(3) diagnostics. T_A_B maps B coordinates into A. No hardware dependencies."""
from dataclasses import dataclass
import math


class CalibrationError(ValueError):
    def __init__(self, code, message):
        self.code = code
        self.message = message
        super().__init__(f'{code}: {message}')


def finite_vector(values, size, label):
    if not isinstance(values, (list, tuple)) or len(values) != size:
        raise CalibrationError('INVALID_VECTOR', f'{label} must contain {size} numbers')
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in values):
        raise CalibrationError('INVALID_VECTOR', f'{label} must contain finite numeric values')
    try:
        converted = tuple(float(v) for v in values)
    except OverflowError as error:
        raise CalibrationError('INVALID_VECTOR', f'{label} cannot be represented as finite floats') from error
    if not all(math.isfinite(v) for v in converted):
        raise CalibrationError('INVALID_VECTOR', f'{label} must contain finite numeric values')
    return converted


def unit_quaternion(values):
    q = finite_vector(values, 4, 'quaternion xyzw')
    if abs(math.sqrt(sum(v * v for v in q)) - 1.0) > 1e-6:
        raise CalibrationError('INVALID_QUATERNION', 'expected unit quaternion; input is not silently normalized')
    return q


def multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz)


def rotate(q, p):
    # Full q*p*q^-1 also behaves correctly within the accepted unit tolerance.
    conjugate = (-q[0], -q[1], -q[2], q[3])
    norm2 = sum(x*x for x in q)
    return tuple(v / norm2 for v in multiply(multiply(q, (*p, 0.0)), conjugate)[:3])


@dataclass(frozen=True)
class Transform:
    translation: tuple
    rotation: tuple

    def __post_init__(self):
        object.__setattr__(self, 'translation', finite_vector(self.translation, 3, 'translation m'))
        object.__setattr__(self, 'rotation', unit_quaternion(self.rotation))

    @classmethod
    def identity(cls):
        return cls((0, 0, 0), (0, 0, 0, 1))

    def apply(self, position):
        rotated = rotate(self.rotation, finite_vector(position, 3, 'position m'))
        return tuple(t + r for t, r in zip(self.translation, rotated))

    def compose(self, other):
        """self=T_A_B, other=T_B_C -> T_A_C, including pose orientation."""
        q = multiply(self.rotation, other.rotation)
        n = math.sqrt(sum(v*v for v in q))
        return Transform(self.apply(other.translation), tuple(v/n for v in q))

    def inverse(self):
        x, y, z, w = self.rotation
        n = math.sqrt(x*x + y*y + z*z + w*w)
        q = (-x/n, -y/n, -z/n, w/n)
        return Transform(rotate(q, tuple(-v for v in self.translation)), q)


def ee_goal_from_tcp(t_base_tcp_goal, t_ee_tcp):
    """Diagnostic T_B_E_goal = T_B_TCP_goal * inverse(T_E_TCP)."""
    return t_base_tcp_goal.compose(t_ee_tcp.inverse())
