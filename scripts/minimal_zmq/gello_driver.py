"""GELLO Dynamixel reader + calibration for the minimal_zmq stack.

Self-contained: depends only on numpy, dynamixel_sdk, and config.py — NOT on
mujoco/dm_control, so the teleop client stays lightweight and importable on any
machine with a GELLO attached.
"""
import threading
import time
from typing import Optional, Sequence

import numpy as np

import config as C

# ============================ Dynamixel reader ============================

_ADDR_PRESENT_POSITION = 132
_LEN_PRESENT_POSITION = 4


class GelloReader:
    """Read-only Dynamixel position reader (no torque — the GELLO servos are
    unpowered; we only read present position). Polls in a background thread;
    get_joints() returns angles in radians.
    """

    def __init__(self, ids: Sequence[int], port: str, baudrate: int = C.BAUDRATE):
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
    """Sim calibration (step 1): (raw - offset)*sign, then modulo-wrap each arm
    joint into its valid band (raw motor angle is only known mod 2*pi at
    power-on)."""
    pos = (np.asarray(raw_arm) - np.asarray(offsets)) * np.asarray(signs)
    c = C.SIM_RANGE_CENTER
    return c + np.mod(pos - c + np.pi, 2 * np.pi) - np.pi


def map_gripper_deg(raw_gripper: float, open_deg: float, close_deg: float) -> float:
    o, c = np.deg2rad(open_deg), np.deg2rad(close_deg)
    return float(np.clip((raw_gripper - o) / (c - o), 0.0, 1.0))


def map_gripper_rad(raw_gripper: float, open_rad: float, close_rad: float) -> float:
    return float(np.clip((raw_gripper - open_rad) / (close_rad - open_rad), 0.0, 1.0))


class IncrementalCalibrator:
    """Real-robot calibration (step 3): normalize once, then track deltas and clip
    to FR3 limits. Supports multi-turn and never emits a target outside the
    robot's joint limits. Mirrors the official ROS2 logic."""

    def __init__(self, assembly_offsets, joint_signs):
        self._offsets = np.asarray(assembly_offsets, dtype=float)
        self._signs = np.asarray(joint_signs, dtype=float)
        self._prev_raw: Optional[np.ndarray] = None
        self._prev: Optional[np.ndarray] = None

    @staticmethod
    def _normalize(raw, offsets, signs):
        return (
            np.mod((raw - offsets) * signs - C.MID_JOINT_POSITIONS, 2 * np.pi)
            - np.pi + C.MID_JOINT_POSITIONS
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
        return np.clip(joints, C.JOINT_POSITION_LIMITS[:, 0], C.JOINT_POSITION_LIMITS[:, 1])


# ============================ leader helpers ============================


def open_gello(arm: str):
    """Open a GelloReader for one arm from config (raises if its port is 'none')."""
    ports = C.current_ports()
    cfg = C.ARMS[arm]
    port = ports.get(arm)
    if port is None or str(port).lower() == "none":
        raise RuntimeError(
            f"arm {arm!r} has port 'none' — set its GELLO port in config.py:PORTS"
        )
    ids = list(cfg["joint_ids"]) + [cfg["gripper_id"]]
    print(f"[gello] {arm}: opening {port} (ids {ids})")
    return GelloReader(ids, port)
