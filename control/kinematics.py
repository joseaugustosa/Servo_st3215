"""Cinemática direta e inversa para braço 4 DOF + garra.

Juntas:
  a1 — base (yaw em Z)
  a2 — ombro (pitch)
  a3 — cotovelo (pitch)
  a4 — pulso (pitch)
  gripper — garra (não entra na IK)
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Tuple


@dataclass
class ArmGeometry:
    L1: float
    L2: float
    L3: float
    L4: float


@dataclass
class JointAngles:
    a1: float = 0.0
    a2: float = 0.0
    a3: float = 0.0
    a4: float = 0.0
    gripper: float = 0.0

    def as_tuple(self) -> Tuple[float, float, float, float, float]:
        return (self.a1, self.a2, self.a3, self.a4, self.gripper)

    def copy(self) -> "JointAngles":
        return JointAngles(*self.as_tuple())


@dataclass
class Pose:
    x: float
    y: float
    z: float
    tool_pitch: float = 0.0


@dataclass
class IKResult:
    ok: bool
    joints: Optional[JointAngles]
    message: str = ""


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def forward_kinematics(geom: ArmGeometry, q: JointAngles) -> Pose:
    a1, a2, a3, a4 = q.a1, q.a2, q.a3, q.a4
    abs2 = a2
    abs3 = a2 + a3
    abs4 = a2 + a3 + a4

    r = (
        geom.L2 * math.cos(abs2)
        + geom.L3 * math.cos(abs3)
        + geom.L4 * math.cos(abs4)
    )
    z = (
        geom.L1
        + geom.L2 * math.sin(abs2)
        + geom.L3 * math.sin(abs3)
        + geom.L4 * math.sin(abs4)
    )
    x = r * math.cos(a1)
    y = r * math.sin(a1)
    return Pose(x=x, y=y, z=z, tool_pitch=abs4)


def joint_positions(geom: ArmGeometry, q: JointAngles):
    a1, a2, a3, a4 = q.a1, q.a2, q.a3, q.a4
    abs2 = a2
    abs3 = a2 + a3
    abs4 = a2 + a3 + a4

    def polar(r: float, z: float):
        return (r * math.cos(a1), r * math.sin(a1), z)

    p0 = (0.0, 0.0, 0.0)
    p1 = (0.0, 0.0, geom.L1)
    r2 = geom.L2 * math.cos(abs2)
    z2 = geom.L1 + geom.L2 * math.sin(abs2)
    p2 = polar(r2, z2)
    r3 = r2 + geom.L3 * math.cos(abs3)
    z3 = z2 + geom.L3 * math.sin(abs3)
    p3 = polar(r3, z3)
    r4 = r3 + geom.L4 * math.cos(abs4)
    z4 = z3 + geom.L4 * math.sin(abs4)
    p4 = polar(r4, z4)
    return [p0, p1, p2, p3, p4]


def inverse_kinematics(
    geom: ArmGeometry,
    target: Pose,
    elbow_up: bool = True,
) -> IKResult:
    x, y, z = target.x, target.y, target.z
    tool_pitch = target.tool_pitch

    a1 = math.atan2(y, x)
    r = math.hypot(x, y)

    rw = r - geom.L4 * math.cos(tool_pitch)
    zw = z - geom.L4 * math.sin(tool_pitch) - geom.L1

    d = math.hypot(rw, zw)
    max_reach = geom.L2 + geom.L3
    min_reach = abs(geom.L2 - geom.L3)

    if d > max_reach + 1e-6:
        return IKResult(False, None, f"Alvo fora de alcance (d={d:.1f} > {max_reach:.1f})")
    if d < min_reach - 1e-6:
        return IKResult(False, None, f"Alvo demasiado perto (d={d:.1f} < {min_reach:.1f})")

    cos_q3 = (geom.L2**2 + geom.L3**2 - d**2) / (2.0 * geom.L2 * geom.L3)
    cos_q3 = clamp(cos_q3, -1.0, 1.0)
    q3_abs = math.acos(cos_q3)
    a3 = -(math.pi - q3_abs) if elbow_up else (math.pi - q3_abs)

    alpha = math.atan2(zw, rw)
    beta = math.acos(
        clamp((geom.L2**2 + d**2 - geom.L3**2) / (2.0 * geom.L2 * d), -1.0, 1.0)
    )
    a2 = alpha + beta if elbow_up else alpha - beta
    a4 = tool_pitch - a2 - a3

    return IKResult(True, JointAngles(a1=a1, a2=a2, a3=a3, a4=a4, gripper=0.0), "OK")
