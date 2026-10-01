"""DH 파라미터 기반 손가락 정기구학. 값은 모두 hand_model.yaml 에서 온다."""

from __future__ import annotations

import math
from typing import Dict, List, Sequence

from .config import DHParam, HandModel

Mat = List[List[float]]


def matmul(a: Mat, b: Mat) -> Mat:
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def dh_transform(p: DHParam, q: float) -> Mat:
    """표준 DH: Rot_z(theta) Trans_z(d) Trans_x(a) Rot_x(alpha)."""
    th = q + p.theta_offset
    ct, st = math.cos(th), math.sin(th)
    ca, sa = math.cos(p.alpha), math.sin(p.alpha)
    return [
        [ct, -st * ca, st * sa, p.a * ct],
        [st, ct * ca, -ct * sa, p.a * st],
        [0.0, sa, ca, p.d],
        [0.0, 0.0, 0.0, 1.0],
    ]


def base_transform(xyz: Sequence[float], rpy: Sequence[float]) -> Mat:
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, xyz[0]],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, xyz[1]],
        [-sp, cp * sr, cp * cr, xyz[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


class HandKinematics:
    def __init__(self, hand: HandModel) -> None:
        self.hand = hand
        self._chains = []
        for f in hand.fingers:
            idx = [hand.joint_index(n) for n in f.joints]
            self._chains.append((f.name, base_transform(f.base_xyz, f.base_rpy), idx))

    def fingertips(self, q: Sequence[float]) -> Dict[str, List[float]]:
        """손바닥 좌표계 기준 각 손가락 끝 위치 [x, y, z] (m)."""
        out = {}
        for name, base, idx in self._chains:
            t = base
            for j in idx:
                t = matmul(t, dh_transform(self.hand.joints[j].dh, q[j]))
            out[name] = [t[0][3], t[1][3], t[2][3]]
        return out
