"""Modelo do braço: estado, limites, conversão para posições de servo."""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List

import math
import yaml

from kinematics import (
    ArmGeometry,
    IKResult,
    JointAngles,
    Pose,
    forward_kinematics,
    inverse_kinematics,
    joint_positions,
)


JOINT_NAMES = ("a1", "a2", "a3", "a4", "gripper")
JOINT_LABELS = {
    "a1": "a1 Base",
    "a2": "a2 Ombro",
    "a3": "a3 Cotovelo",
    "a4": "a4 Pulso",
    "gripper": "Garra",
}


class ArmController:
    def __init__(self, config_path: str | Path | None = None):
        if config_path is None:
            config_path = Path(__file__).with_name("config.yaml")
        self.config_path = Path(config_path)
        with open(self.config_path, "r", encoding="utf-8") as f:
            self.cfg = yaml.safe_load(f)
        self._apply_cfg()
        self.elbow_up = True
        self.joints = JointAngles(
            a1=0.0,
            a2=math.radians(45),
            a3=math.radians(-60),
            a4=math.radians(15),
            gripper=0.0,
        )
        self.pose = forward_kinematics(self.geom, self.joints)
        self.last_ik_message = "Pronto"

    def _apply_cfg(self) -> None:
        a = self.cfg["arm"]
        self.geom = ArmGeometry(float(a["L1"]), float(a["L2"]), float(a["L3"]), float(a["L4"]))
        self.limits = {
            name: (math.radians(lo), math.radians(hi))
            for name, (lo, hi) in a["joint_limits_deg"].items()
        }
        self.servo_ids: Dict[str, int] = {k: int(v) for k, v in a["servo_ids"].items()}
        self.servo_center = int(a["servo_center"])
        self.counts_per_deg = float(a["counts_per_deg"])
        self.joint_sign = {k: int(v) for k, v in a["joint_sign"].items()}
        self.joint_offset_deg = {k: float(v) for k, v in a["joint_offset_deg"].items()}
        self.default_speed = int(a["default_speed"])
        self.default_acc = int(a["default_acc"])

    def save_config(self) -> None:
        with open(self.config_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(self.cfg, f, sort_keys=False, allow_unicode=True)

    def update_geometry(self, L1: float, L2: float, L3: float, L4: float) -> None:
        self.cfg["arm"]["L1"] = float(L1)
        self.cfg["arm"]["L2"] = float(L2)
        self.cfg["arm"]["L3"] = float(L3)
        self.cfg["arm"]["L4"] = float(L4)
        self.geom = ArmGeometry(float(L1), float(L2), float(L3), float(L4))
        self.pose = forward_kinematics(self.geom, self.joints)

    def update_servo_ids(self, ids: Dict[str, int]) -> None:
        for k, v in ids.items():
            self.cfg["arm"]["servo_ids"][k] = int(v)
            self.servo_ids[k] = int(v)

    def update_offsets(self, offsets: Dict[str, float]) -> None:
        for k, v in offsets.items():
            self.cfg["arm"]["joint_offset_deg"][k] = float(v)
            self.joint_offset_deg[k] = float(v)

    def update_signs(self, signs: Dict[str, int]) -> None:
        for k, v in signs.items():
            self.cfg["arm"]["joint_sign"][k] = int(v)
            self.joint_sign[k] = int(v)

    def clamp_joints(self, q: JointAngles) -> JointAngles:
        out = q.copy()
        for name in JOINT_NAMES:
            lo, hi = self.limits[name]
            setattr(out, name, max(lo, min(hi, getattr(out, name))))
        return out

    def set_joints(self, q: JointAngles) -> JointAngles:
        self.joints = self.clamp_joints(q)
        self.pose = forward_kinematics(self.geom, self.joints)
        self.last_ik_message = "Modo articular"
        return self.joints

    def set_gripper(self, angle_rad: float) -> None:
        lo, hi = self.limits["gripper"]
        self.joints.gripper = max(lo, min(hi, angle_rad))

    def move_to_pose(self, pose: Pose) -> IKResult:
        result = inverse_kinematics(self.geom, pose, elbow_up=self.elbow_up)
        if not result.ok or result.joints is None:
            self.last_ik_message = result.message
            return result
        q = result.joints
        q.gripper = self.joints.gripper
        self.joints = self.clamp_joints(q)
        self.pose = forward_kinematics(self.geom, self.joints)
        self.last_ik_message = "IK OK"
        return IKResult(True, self.joints, "IK OK")

    def link_points(self):
        return joint_positions(self.geom, self.joints)

    def angle_to_servo_counts(self, joint_name: str, angle_rad: float) -> int:
        deg = math.degrees(angle_rad) + self.joint_offset_deg[joint_name]
        counts = self.servo_center + self.joint_sign[joint_name] * deg * self.counts_per_deg
        return int(max(0, min(4095, round(counts))))

    def servo_command_list(self) -> List[dict]:
        cmds = []
        for name in JOINT_NAMES:
            cmds.append(
                {
                    "name": name,
                    "id": self.servo_ids[name],
                    "pos": self.angle_to_servo_counts(name, getattr(self.joints, name)),
                    "speed": self.default_speed,
                    "acc": self.default_acc,
                }
            )
        return cmds

    def joints_deg(self) -> Dict[str, float]:
        return {n: math.degrees(getattr(self.joints, n)) for n in JOINT_NAMES}
