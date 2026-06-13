"""Self-contained MuJoCo dual-arm Panda scene for the minimal_zmq stack.

Lifted from scripts/minimal/gello_mujoco.py — depends only on mujoco, dm_control,
numpy, and config.py. Builds a two-Panda scene at the real install transforms and
resolves qpos/actuator addresses BY NAME (left/joint1.., the tendon gripper
actuator) so the right arm is robust to dm_control's attach ordering — a flat
index assumption is what made the right arm drive the wrong joints before.
"""
from typing import Dict

import numpy as np

import config as C


def _attach_hand_to_arm(arm_mjcf, hand_mjcf) -> None:
    from dm_control import mjcf

    physics = mjcf.Physics.from_mjcf_model(hand_mjcf)
    attachment_site = arm_mjcf.find("site", "attachment_site")
    if attachment_site is None:
        raise ValueError("No attachment site found in the arm model.")
    arm_key = arm_mjcf.find("key", "home")
    if arm_key is not None:
        hand_key = hand_mjcf.find("key", "home")
        if hand_key is None:
            arm_key.ctrl = np.concatenate([arm_key.ctrl, np.zeros(physics.model.nu)])
            arm_key.qpos = np.concatenate([arm_key.qpos, np.zeros(physics.model.nq)])
        else:
            arm_key.ctrl = np.concatenate([arm_key.ctrl, hand_key.ctrl])
            arm_key.qpos = np.concatenate([arm_key.qpos, hand_key.qpos])
    attachment_site.attach(hand_mjcf)


def _add_floor_and_light(arena) -> None:
    arena.asset.add(
        "texture", name="groundplane", type="2d", builtin="checker",
        rgb1=[0.2, 0.3, 0.4], rgb2=[0.1, 0.2, 0.3], width=300, height=300,
        mark="edge", markrgb=[0.8, 0.8, 0.8],
    )
    arena.asset.add(
        "material", name="groundplane", texture="groundplane",
        texuniform=True, texrepeat=[5, 5], reflectance=0.2,
    )
    arena.worldbody.add(
        "geom", name="floor", type="plane", size=[0, 0, 0.05], material="groundplane",
    )
    arena.worldbody.add("light", pos=[0, 0, 1.5], dir=[0, 0, -1], directional=True)


def _attach_arm(arena, xml_path, quat=None, pos=None, name=None, gripper_xml=None):
    from dm_control import mjcf

    arm = mjcf.from_path(xml_path)
    if name is not None:
        arm.model = name  # namespace so dual arms don't collide on element names
    if gripper_xml is not None:
        _attach_hand_to_arm(arm, mjcf.from_path(gripper_xml))
    frame = arena.worldbody.attach(arm)
    if quat is not None:
        frame.quat = list(quat)
    if pos is not None:
        frame.pos = list(pos)
    return arm


def build_dual_arm_scene():
    """Two Pandas at the real install transforms. panda.xml already includes the
    hand (tendon-driven gripper), so no separate gripper attachment."""
    from dm_control import mjcf

    arena = mjcf.RootElement()
    _add_floor_and_light(arena)
    for name in ("left", "right"):
        _attach_arm(
            arena, C.PANDA_XML,
            quat=C.SIM_ARM_BASE[name]["quat"], pos=C.SIM_ARM_BASE[name]["pos"], name=name,
        )
    return arena


class DualArmSim:
    """Builds the model and resolves, per arm and BY NAME, the qpos addresses and
    actuator indices we read/write."""

    def __init__(self):
        import mujoco

        arena = build_dual_arm_scene()
        assets: Dict[str, str] = {}
        for asset in arena.asset.all_children():
            if asset.tag == "mesh":
                assets[asset.file.get_vfs_filename()] = asset.file.contents
        xml_string = arena.to_xml_string()

        self.model = mujoco.MjModel.from_xml_string(xml_string, assets)
        self.data = mujoco.MjData(self.model)
        self.nu = self.model.nu

        def jadr(jname: str) -> int:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jname)
            if jid < 0:
                raise RuntimeError(f"joint {jname!r} not found in model")
            return int(self.model.jnt_qposadr[jid])

        def act_id(aname: str) -> int:
            aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, aname)
            if aid < 0:
                raise RuntimeError(f"actuator {aname!r} not found in model")
            return int(aid)

        self.arm_qadr: Dict[str, np.ndarray] = {}
        self.arm_act: Dict[str, np.ndarray] = {}
        self.grip_act: Dict[str, int] = {}
        self.grip_qadr: Dict[str, int] = {}
        for name in ("left", "right"):
            self.arm_qadr[name] = np.array(
                [jadr(f"{name}/joint{i}") for i in range(1, 8)], dtype=int)
            self.arm_act[name] = np.array(
                [act_id(f"{name}/actuator{i}") for i in range(1, 8)], dtype=int)
            self.grip_act[name] = act_id(f"{name}/actuator8")  # tendon gripper
            self.grip_qadr[name] = jadr(f"{name}/finger_joint1")

    def set_command(self, cmds: Dict[str, np.ndarray]) -> None:
        """cmds = {"left": [j1..j7, grip(0-1)], "right": [...]}. Writes each arm's
        7 joint targets + gripper (scaled to the 0-255 ctrlrange) into data.ctrl."""
        for name, vec in cmds.items():
            vec = np.asarray(vec, dtype=float)
            self.data.ctrl[self.arm_act[name]] = vec[:7]
            self.data.ctrl[self.grip_act[name]] = float(vec[7]) * 255.0

    def read_arm(self, name: str) -> np.ndarray:
        """Current [q(7), grip(0-1)] of one arm, read back by name."""
        q = self.data.qpos[self.arm_qadr[name]].copy()
        grip = float(np.clip(self.data.qpos[self.grip_qadr[name]] / 0.04, 0.0, 1.0))
        return np.concatenate([q, [grip]])

    def write_arm_qpos(self, name: str, q_arm: np.ndarray) -> None:
        """Directly set one arm's 7 joint positions (kinematic mirror, no step)."""
        self.data.qpos[self.arm_qadr[name]] = np.asarray(q_arm, dtype=float)[:7]
