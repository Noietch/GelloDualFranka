"""Minimal self-contained GELLO core: Dynamixel reading, calibration, and a
dual-arm MuJoCo scene — with NO dependency on the `gello` package, ZMQ, or the
dev container. Only needs: numpy, dynamixel_sdk, mujoco, dm_control.

This module is a library. The CLI lives in main.py (three modes: teleop-sim,
sync-robot, teleop-robot). Edit the CONFIG block below to match your hardware.

Two calibration conventions are kept side by side, because the sim path and the
real path were calibrated separately and use different sign conventions:
  - sim  (mode teleop-sim): modulo-wrap,  pos = (raw - sim_offsets) * sim_signs
  - real (mode teleop-robot): incremental delta + FR3 joint-limit clip
Using the wrong one on the real robot drives joints past their limits — the
joint4-frozen-at-3.077 symptom. Keep them distinct.
"""
import platform
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import numpy as np

# ============================ CONFIG (edit here) ============================

_REPO_ROOT = Path(__file__).resolve().parent.parent.parent
PANDA_XML = str(
    _REPO_ROOT / "third_party" / "mujoco_menagerie" / "franka_emika_panda" / "panda.xml"
)

# Dual-arm base install transforms (MuJoCo quat wxyz + pos), from the real rig's
# urdf/kinematics.yaml. Authoritative — do not re-derive by mirroring.
SIM_ARM_BASE = {
    "left":  {"quat": [0.865807, -0.436878, 0.022288, -0.242939],
              "pos":  [0.036395,  0.050681, 0.0508858]},
    "right": {"quat": [0.865807,  0.436878, 0.022288,  0.242939],
              "pos":  [0.036395, -0.050681, 0.0508858]},
}

# Serial ports per platform. macOS names are unstable across replug — run
# `ls /dev/cu.usbserial*` and update. Linux by-id names are stable (recommended).
# "none" = arm shown in sim / skipped on real, but not controlled by a GELLO.
PORTS = {
    "Darwin": {"left": "none", "right": "/dev/cu.usbserial-10"},
    "Linux":  {"left": "none",
               "right": "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"},
}

# Per-arm calibration. joint_ids = 7 arm servos; gripper_id = the gripper servo.
# sim_*  : from configs/bimanual.yaml      (modulo-wrap path)
# real_* : from GELLO calibration results  (incremental + limit-clip path)
# gripper_deg = (open_deg, close_deg) for the sim path; gripper_range_rad =
# [open_rad, close_rad] for the real path.
ARMS = {
    "left": {
        "joint_ids": [1, 2, 3, 4, 5, 6, 7],
        "gripper_id": 8,
        "sim_offsets": [1.571, 4.712, 4.712, 0.0, 3.142, 4.712, 3.142],
        "sim_signs":   [1, 1, 1, 1, 1, -1, 1],
        "gripper_deg": [132, 202],
        "real_offsets": [3.142, 3.142, 4.712, 4.712, 3.142, 1.571, 3.142],
        "real_signs":   [1, -1, 1, 1, 1, 1, 1],
        "gripper_range_rad": [2.317, 3.537],
    },
    "right": {
        "joint_ids": [1, 2, 3, 4, 5, 6, 7],
        "gripper_id": 8,
        "sim_offsets": [3.0787, 3.1355, -1.5432, 4.6098, 3.117, 4.7387, 9.3557],
        "sim_signs":   [1, 1, 1, 1, 1, -1, 1],
        "gripper_deg": [145, 189],  # measured open/close range (raw motor deg)
        "real_offsets": [1.571, 4.712, 1.571, 0.0, 3.142, 1.571, 3.142],
        "real_signs":   [1, -1, 1, 1, 1, 1, 1],
        "gripper_range_rad": [2.299, 3.519],
    },
}
BAUDRATE = 57600
HZ_SIM = 60      # sim teleop loop rate
HZ_ROS = 25      # real-robot publish rate

# FR3 home pose (7 arm joints) — an uncontrolled ("none") arm holds this.
FR3_HOME = [0.0, 0.0, 0.0, -1.57079, 0.0, 1.57079, -0.7853]

# ROS2 topics (mode sync-robot subscribes / mode teleop-robot publishes).
# {ns} is filled with the arm namespace ("left"/"right").
ROBOT_STATE_TOPIC = "/franka/joint_states"   # subscribe (sync-robot)
GELLO_PUB_TOPIC   = "/gello/joint_states"     # publish   (teleop-robot)
GRIPPER_TOPIC     = "/gripper/gripper_client/target_gripper_width_percent"
FR3_JOINT_NAMES = [f"fr3_joint{i}" for i in range(1, 8)]

# ======================= FR3 constants (real path) =======================
# From https://frankarobotics.github.io/docs/robot_specifications.html
JOINT_POSITION_LIMITS = np.array([
    [-2.9007, 2.9007],
    [-1.8361, 1.8361],
    [-2.9007, 2.9007],
    [-3.0770, -0.1169],
    [-2.8763, 2.8763],
    [0.4398, 4.6216],
    [-3.0508, 3.0508],
])
MID_JOINT_POSITIONS = JOINT_POSITION_LIMITS.mean(axis=1)

# Centers of each arm joint's valid range, for modulo-wrap (sim path). FR3 7-DOF.
SIM_RANGE_CENTER = np.array([0.0, 0.0, 0.0, -1.571, 0.0, 1.866, 0.0])


def current_ports() -> Dict[str, str]:
    """Serial ports for this OS (raises if the platform isn't configured)."""
    sysname = platform.system()
    if sysname not in PORTS:
        raise RuntimeError(f"No PORTS entry for platform {sysname!r}; edit CONFIG.")
    return PORTS[sysname]


# ============================ Dynamixel reader ============================

_ADDR_PRESENT_POSITION = 132
_LEN_PRESENT_POSITION = 4


class GelloReader:
    """Read-only Dynamixel position reader (no torque, no current control).

    Polls present position for `ids` in a background thread; get_joints() returns
    angles in radians. Deliberately omits the lsof/fuser/sudo port-stealing logic
    of the full driver — keep it simple; if the port is busy, fix it manually.
    """

    def __init__(self, ids: Sequence[int], port: str, baudrate: int = BAUDRATE):
        from dynamixel_sdk.group_sync_read import GroupSyncRead
        from dynamixel_sdk.packet_handler import PacketHandler
        from dynamixel_sdk.port_handler import PortHandler

        self._ids = list(ids)
        self._port = port
        self._angles: Optional[np.ndarray] = None
        self._lock = threading.Lock()
        self._stop = threading.Event()

        self._port_handler = PortHandler(port)
        self._packet_handler = PacketHandler(2.0)
        self._reader = GroupSyncRead(
            self._port_handler, self._packet_handler,
            _ADDR_PRESENT_POSITION, _LEN_PRESENT_POSITION,
        )
        if not self._port_handler.openPort():
            raise ConnectionError(f"Failed to open port {port}")
        if not self._port_handler.setBaudRate(baudrate):
            raise ConnectionError(f"Failed to set baudrate {baudrate} on {port}")
        for dxl_id in self._ids:
            if not self._reader.addParam(dxl_id):
                raise ConnectionError(f"Failed to add servo id {dxl_id} on {port}")

        self._thread = threading.Thread(target=self._poll, daemon=True)
        self._thread.start()

    def _poll(self) -> None:
        from dynamixel_sdk.robotis_def import COMM_SUCCESS

        while not self._stop.is_set():
            time.sleep(0.001)
            with self._lock:
                if self._reader.txRxPacket() != COMM_SUCCESS:
                    continue
                vals = np.zeros(len(self._ids), dtype=int)
                ok = True
                for i, dxl_id in enumerate(self._ids):
                    if not self._reader.isAvailable(
                        dxl_id, _ADDR_PRESENT_POSITION, _LEN_PRESENT_POSITION
                    ):
                        ok = False
                        break
                    raw = self._reader.getData(
                        dxl_id, _ADDR_PRESENT_POSITION, _LEN_PRESENT_POSITION
                    )
                    if raw > 0x7FFFFFFF:  # 32-bit two's complement
                        raw -= 0x100000000
                    vals[i] = raw
                if ok:
                    self._angles = vals

    def get_joints(self) -> np.ndarray:
        """Current angles (radians), one per id. Blocks until first read."""
        while self._angles is None:
            time.sleep(0.05)
        with self._lock:
            return self._angles.copy() / 2048.0 * np.pi

    def close(self) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._port_handler.closePort()


# ============================ Calibration ============================

def calib_sim(raw_arm: np.ndarray, offsets, signs) -> np.ndarray:
    """Sim calibration: (raw - offset)*sign, then modulo-wrap each arm joint into
    its valid band (raw motor angle is only known mod 2*pi at power-on)."""
    pos = (np.asarray(raw_arm) - np.asarray(offsets)) * np.asarray(signs)
    c = SIM_RANGE_CENTER
    return c + np.mod(pos - c + np.pi, 2 * np.pi) - np.pi


def map_gripper_deg(raw_gripper: float, open_deg: float, close_deg: float) -> float:
    o, c = np.deg2rad(open_deg), np.deg2rad(close_deg)
    return float(np.clip((raw_gripper - o) / (c - o), 0.0, 1.0))


def map_gripper_rad(raw_gripper: float, open_rad: float, close_rad: float) -> float:
    return float(np.clip((raw_gripper - open_rad) / (close_rad - open_rad), 0.0, 1.0))


class IncrementalCalibrator:
    """Real-robot calibration: normalize once, then track deltas and clip to FR3
    limits. Supports multi-turn (joint may rotate past +-pi) and never emits a
    target outside the robot's joint limits. Mirrors the official ROS2 logic."""

    def __init__(self, assembly_offsets, joint_signs):
        self._offsets = np.asarray(assembly_offsets, dtype=float)
        self._signs = np.asarray(joint_signs, dtype=float)
        self._prev_raw: Optional[np.ndarray] = None
        self._prev: Optional[np.ndarray] = None

    @staticmethod
    def _normalize(raw, offsets, signs):
        return (
            np.mod((raw - offsets) * signs - MID_JOINT_POSITIONS, 2 * np.pi)
            - np.pi + MID_JOINT_POSITIONS
        )

    def process(self, raw_arm: np.ndarray) -> np.ndarray:
        raw_arm = np.asarray(raw_arm, dtype=float)
        if self._prev is None:
            self._prev = self._normalize(raw_arm, self._offsets, self._signs)
            self._prev_raw = raw_arm.copy()
        delta = (raw_arm - self._prev_raw) * self._signs
        joints = self._prev + delta
        self._prev = joints.copy()
        self._prev_raw = raw_arm.copy()
        return np.clip(joints, JOINT_POSITION_LIMITS[:, 0], JOINT_POSITION_LIMITS[:, 1])


# ============================ MuJoCo dual-arm scene ============================

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
    """Assemble a two-Panda scene at the real install transforms. panda.xml
    already includes the hand (tendon-driven gripper), so no separate gripper."""
    from dm_control import mjcf

    arena = mjcf.RootElement()
    _add_floor_and_light(arena)
    for name in ("left", "right"):
        _attach_arm(
            arena, PANDA_XML,
            quat=SIM_ARM_BASE[name]["quat"], pos=SIM_ARM_BASE[name]["pos"], name=name,
        )
    return arena


class DualArmSim:
    """Builds the model and resolves, per arm and by NAME, the qpos addresses and
    actuator indices we read/write. Resolving by name (left/joint1.. , the tendon
    gripper actuator) instead of assuming actuator order makes the right arm
    robust to dm_control's attach/ordering — a flat index assumption is what made
    the right arm drive the wrong joints before."""

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

        # Per arm: 7 arm-joint qpos addresses, 7 arm actuator indices, gripper
        # actuator index, and the gripper finger qpos address. All by name.
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
        7 joint targets + gripper (scaled to the 0-255 ctrlrange) into data.ctrl
        at the right actuator indices — no ordering assumptions."""
        for name, vec in cmds.items():
            vec = np.asarray(vec, dtype=float)
            self.data.ctrl[self.arm_act[name]] = vec[:7]
            self.data.ctrl[self.grip_act[name]] = float(vec[7]) * 255.0

    def write_arm_qpos(self, name: str, q_arm: np.ndarray) -> None:
        """Directly set one arm's 7 joint positions (kinematic mirror, no step)."""
        self.data.qpos[self.arm_qadr[name]] = np.asarray(q_arm, dtype=float)
