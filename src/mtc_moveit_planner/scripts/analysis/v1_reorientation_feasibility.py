#!/usr/bin/env python3
"""V1 预对准的离线可行性分析（纯 numpy/scipy，不连接 ROS，不移动机器人）。

!! 已被取代：本脚本假设"插入方向 = 法兰轴线"（旧占位 TCP）。真实舌规（舌规V1.STEP）的舌头
垂直于法兰轴线，结论不适用，见 docs/planning/V1_FEASIBILITY.md。仅作历史记录保留。

从展开后的 URDF 计算 base_link -> ee_link 正运动学，回答两个问题：
  1. 从起点（默认 SRDF home）到预对准位姿，末端至少要转多少度？在 0.1 rad/s 角速度上限下
     物理下限是多少秒？（与规划算法无关）
  2. 沿"末端直线 + 姿态匀速插值（slerp）"的理想路径，从起点连续求逆解能否走通？
     卡在哪个关节限位？可行时，末端限速下的时长下界是多少？

用法：
  python3 v1_reorientation_feasibility.py <expanded.urdf>            # 默认占位几何
  python3 v1_reorientation_feasibility.py <expanded.urdf> --grid     # 多种提环位置/插入方向/滚转
依赖：python3-numpy python3-scipy
"""
import argparse
import itertools
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

HOME = np.array([0.0, -0.0334, 1.236, -0.3675, 1.5701, 0.0])          # SRDF home（与 move_group 一致）
# 控制器限位（foreArm/wrist1/wrist3 比 URDF 窄）；其余取 ±2π。
LIMITS = np.array([2 * np.pi, 2 * np.pi, 2.41, 2.66, 2 * np.pi, 2.23])
V_LIN, W_ANG, V_JOINT = 0.02, 0.1, 3.15 * 0.1                           # 原 side_insert 上限与关节速度 × 0.1


class Chain:
    def __init__(self, urdf_path, base='base_link', tip='ee_link'):
        root = ET.parse(urdf_path).getroot()
        by_child = {}
        for j in root.findall('joint'):
            o = j.find('origin')
            ax = j.find('axis')
            by_child[j.find('child').get('link')] = dict(
                name=j.get('name'), type=j.get('type'), parent=j.find('parent').get('link'),
                xyz=np.array([float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]),
                rpy=[float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()],
                axis=np.array([float(v) for v in ax.get('xyz').split()]) if ax is not None else np.array([0, 0, 1.0]))
        chain, link = [], tip
        while link != base:
            chain.append(by_child[link])
            link = by_child[link]['parent']
        self.joints = chain[::-1]

    def fk(self, q):
        T, k = np.eye(4), 0
        for j in self.joints:
            A = np.eye(4)
            A[:3, :3] = R.from_euler('xyz', j['rpy']).as_matrix()
            A[:3, 3] = j['xyz']
            T = T @ A
            if j['type'] == 'revolute':
                B = np.eye(4)
                B[:3, :3] = R.from_rotvec(j['axis'] * q[k]).as_matrix()
                T, k = T @ B, k + 1
        return T

    def jac(self, q, eps=1e-6):
        J, T0 = np.zeros((6, 6)), self.fk(q)
        for i in range(6):
            dq = q.copy()
            dq[i] += eps
            T1 = self.fk(dq)
            J[:3, i] = (T1[:3, 3] - T0[:3, 3]) / eps
            J[3:, i] = (R.from_matrix(T1[:3, :3]) * R.from_matrix(T0[:3, :3]).inv()).as_rotvec() / eps
        return J

    def ik(self, q, p, Rg, iters=100, lam=0.01):
        for _ in range(iters):
            T = self.fk(q)
            e = np.concatenate([p - T[:3, 3], (Rg * R.from_matrix(T[:3, :3]).inv()).as_rotvec()])
            if np.linalg.norm(e[:3]) < 1e-5 and np.linalg.norm(e[3:]) < 1e-4:
                return q, True
            J = self.jac(q)
            q = q + J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(6), e)
        return q, False


def hook_up_rotation(insert_dir, roll_deg=0.0):
    """TCP z = 插入方向；TCP x = 尽量朝世界上方（钩口朝上）；再绕 z 滚转 roll_deg。"""
    z = np.asarray(insert_dir, float) / np.linalg.norm(insert_dir)
    up = np.array([0.0, 0.0, 1.0])
    x = up - z * (up @ z)
    x /= np.linalg.norm(x)
    return R.from_matrix(np.column_stack([x, np.cross(z, x), z])) * R.from_euler('z', np.radians(roll_deg))


def track(chain, goal_p, goal_R, start=HOME, n=120):
    """沿末端直线 + slerp 从 start 连续求逆解；返回 (时长下界 s 或 None, 走到的比例, 卡住原因)。"""
    T0 = chain.fk(start)
    p0, R0 = T0[:3, 3], R.from_matrix(T0[:3, :3])
    sl = Slerp([0, 1], R.concatenate([R0, goal_R]))
    q, t, pp, pr = start.copy(), 0.0, p0, R0
    for k in range(1, n + 1):
        s = k / n
        p, Rk = (1 - s) * p0 + s * goal_p, sl(s)
        qn, ok = chain.ik(q.copy(), p, Rk)
        over = [chain.joints[i]['name'] for i in range(6) if abs(qn[i]) > LIMITS[i]]
        if not ok or np.max(np.abs(qn - q)) > 0.2 or over:
            why = 'joint limit ' + ','.join(over) if over else ('IK diverged' if not ok else 'joint jump (singularity)')
            return None, s, why
        t += max(np.linalg.norm(p - pp) / V_LIN, (Rk * pr.inv()).magnitude() / W_ANG,
                 np.max(np.abs(qn - q)) / V_JOINT)
        q, pp, pr = qn, p, Rk
    return t, 1.0, ''


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('urdf')
    ap.add_argument('--grid', action='store_true')
    args = ap.parse_args()
    chain = Chain(args.urdf)
    T0 = chain.fk(HOME)
    R0 = R.from_matrix(T0[:3, :3])
    print('home ee position %s, tool axis %s' % (np.round(T0[:3, 3], 3), np.round(T0[:3, 2], 3)))

    # 当前占位：提环 (0.40,-0.05,0.13)，quat (0,0.7071,0,0.7071)，TCP 单位变换，预对准退 0.03 m。
    ring_p = np.array([0.40, -0.05, 0.13])
    ring_R = R.from_quat([0, 0.70710678, 0, 0.70710678])
    goal_p = ring_p - 0.03 * ring_R.as_matrix()[:, 2]
    for label, gR in [('placeholder ring/TCP roll', ring_R),
                      ('same, hook rolled 180 deg about its axis', ring_R * R.from_euler('z', np.pi))]:
        ang = (gR * R0.inv()).magnitude()
        t, frac, why = track(chain, goal_p, gR)
        print('%-42s rotation %5.1f deg -> >= %4.1f s at 0.1 rad/s; straight path from home: %s' % (
            label, np.degrees(ang), ang / W_ANG,
            ('%.1f s (velocity bound)' % t) if t else 'blocked at %.0f%% (%s)' % (100 * frac, why)))

    if args.grid:
        print('\nring xyz              insert dir  roll  rotation  straight path from home')
        dirs = {'+X': (1, 0, 0), '-X': (-1, 0, 0), '+Y': (0, 1, 0), '-Y': (0, -1, 0)}
        rings = [(x, y, z) for x in (0.40, 0.50) for y in (-0.15, 0.05) for z in (0.13, 0.25)]
        for ring, (dn, d), roll in itertools.product(rings, dirs.items(), (0, 180)):
            gR = hook_up_rotation(d, roll)
            gp = np.array(ring) - 0.03 * np.asarray(d, float)
            t, frac, why = track(chain, gp, gR)
            print('%-21s %-10s %4d  %6.1f   %s' % (str(ring), dn, roll, np.degrees((gR * R0.inv()).magnitude()),
                  ('%.1f s' % t) if t else 'blocked at %.0f%% (%s)' % (100 * frac, why)))


if __name__ == '__main__':
    main()
