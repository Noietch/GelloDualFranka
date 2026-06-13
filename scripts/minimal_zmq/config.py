"""Standalone config for the minimal_zmq control stack.

Self-contained: NO dependency on scripts/minimal or the gello package. Holds the
hardware/calibration/network constants shared by the GELLO driver, the MuJoCo
sim follower, and the franky real follower.

Two calibration conventions are kept side by side (see gello_driver.py):
  - sim  (step 1, teleop into MuJoCo): modulo-wrap, pos = (raw - sim_off)*sim_sign
  - real (step 3, teleop into franky):  incremental delta + FR3 joint-limit clip
Using the wrong one on the real robot drives joints past their limits (the
joint4-frozen-at-3.077 symptom). The step scripts pass --calib to pick correctly.
"""
import platform
from pathlib import Path
from typing import Dict

import numpy as np

# Repo root is three levels up: scripts/minimal_zmq/config.py -> repo/
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

# GELLO serial ports per platform. macOS names are unstable across replug — run
# `ls /dev/cu.usbserial*` and update. Linux by-id names are stable (recommended).
# "none" = arm not driven by a GELLO (held at home in sim).
PORTS = {
    "Darwin": {"left": "none", "right": "/dev/cu.usbserial-10"},
    "Linux":  {"left": "/dev/serial/by-id/<FILL_IN_LEFT_GELLO>",
               "right": "/dev/serial/by-id/<FILL_IN_RIGHT_GELLO>"},
}

# Per-arm config. joint_ids = 7 arm servos; gripper_id = the gripper servo.
#   sim_*   : modulo-wrap path (step 1, into MuJoCo)
#   real_*  : incremental + limit-clip path (step 3, into franky)
#   gripper_deg       = (open_deg, close_deg) for the sim path
#   gripper_range_rad = [open_rad, close_rad] for the real path
#   robot_ip          : FR3 control box IP (from ros2 example_fr3_duo_config.yaml)
#   robot_zmq_port    : the follower's ZMQ port for this arm (unique per arm)
#   grip_width_rng    : Franka Hand finger travel [closed_m, open_m]
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
        "robot_ip": "192.168.20.11",
        "robot_zmq_port": 6000,
        "grip_width_rng": [0.0, 0.08],
    },
    "right": {
        "joint_ids": [1, 2, 3, 4, 5, 6, 7],
        "gripper_id": 8,
        "sim_offsets": [3.0787, 3.1355, -1.5432, 4.6098, 3.117, 4.7387, 9.3557],
        "sim_signs":   [1, 1, 1, 1, 1, -1, 1],
        "gripper_deg": [145, 189],
        "real_offsets": [1.571, 4.712, 1.571, 0.0, 3.142, 1.571, 3.142],
        "real_signs":   [1, -1, 1, 1, 1, 1, 1],
        "gripper_range_rad": [2.299, 3.519],
        "robot_ip": "192.168.20.12",
        "robot_zmq_port": 6001,
        "grip_width_rng": [0.0, 0.08],
    },
}

BAUDRATE = 57600
HZ = 60           # teleop / viewer loop rate

# FR3 home pose (7 arm joints) — an uncontrolled arm holds this in sim.
FR3_HOME = [0.0, 0.0, 0.0, -1.57079, 0.0, 1.57079, -0.7853]

# FR3 joint position limits, https://frankarobotics.github.io/docs/robot_specifications.html
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
    """GELLO serial ports for this OS (raises if the platform isn't configured)."""
    sysname = platform.system()
    if sysname not in PORTS:
        raise RuntimeError(f"No PORTS entry for platform {sysname!r}; edit config.py.")
    return PORTS[sysname]
