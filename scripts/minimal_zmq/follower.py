"""franky real-robot follower (steps 2 & 3). Serves the follower protocol over ZMQ.

    python follower.py --arm right                 # control mode (step 3)
    python follower.py --arm right --read-only      # state only, ignores commands (step 2)

franky is Linux + PREEMPT_RT only and is imported lazily, so this file imports
fine anywhere; it only fails if you actually construct FrankyFollower without
franky installed.

Streaming model (verified): franky has no servo primitive — to follow a live
target you repeatedly issue an asynchronous JointMotion; Ruckig blends each new
target into the current motion and a later async move preempts the previous one.
So command_joint_state() fires one async move per call. The control mode must not
change while preempting, so this path only ever issues JointMotion.

--read-only is the safe step-2 mode: it serves real joint state for the MuJoCo
mirror (mirror.py) but drops any command, so connecting can't move the robot.

Gripper convention (matches teleop.py / the sim follower): command g in [0,1],
0 = open, 1 = closed; width = open_m*(1-g) + closed_m*g.
"""
import argparse
from typing import Dict, Optional

import numpy as np

import config as C
from zmq_transport import ZMQRobotServer

NUM_DOFS = 8  # 7 arm joints + 1 gripper


def _grip_to_width(g: float, closed_m: float, open_m: float) -> float:
    g = float(np.clip(g, 0.0, 1.0))
    return open_m * (1.0 - g) + closed_m * g


def _width_to_grip(width: float, closed_m: float, open_m: float) -> float:
    span = open_m - closed_m
    return 0.0 if abs(span) < 1e-9 else float(np.clip((open_m - width) / span, 0.0, 1.0))


class FrankyFollower:
    """FR3 follower via franky. franky is imported lazily in __init__."""

    def __init__(self, robot_ip: str, grip_width_rng, read_only: bool = False,
                 relative_dynamics: float = 0.08, grip_speed: float = 0.1,
                 grip_eps_m: float = 0.005):
        import franky

        self._franky = franky
        self._read_only = read_only
        self._closed_m, self._open_m = float(grip_width_rng[0]), float(grip_width_rng[1])
        self._grip_speed = grip_speed
        self._grip_eps_m = grip_eps_m

        print(f"[franky] connecting robot {robot_ip}  (read_only={read_only})")
        self._robot = franky.Robot(robot_ip)
        self._robot.relative_dynamics_factor = relative_dynamics
        self._robot.recover_from_errors()
        self._gripper = franky.Gripper(robot_ip)
        self._last_grip_width: Optional[float] = None
        self._grip_future = None
        print(f"[franky] ready (relative_dynamics={relative_dynamics})")

    def num_dofs(self) -> int:
        return NUM_DOFS

    def get_joint_state(self) -> np.ndarray:
        q = np.asarray(self._robot.current_joint_state.position, dtype=float)[:7]
        grip = _width_to_grip(self._gripper.width, self._closed_m, self._open_m)
        return np.concatenate([q, [grip]])

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        if self._read_only:
            return  # step 2: never move the robot
        joint_state = np.asarray(joint_state, dtype=float)
        try:
            self._robot.recover_from_errors()  # async errors surface on next move
            self._robot.move(
                self._franky.JointMotion(joint_state[:7].tolist()), asynchronous=True)
        except Exception as e:
            print(f"\n[franky] move error (recovering): {e}")
            try:
                self._robot.recover_from_errors()
            except Exception:
                pass

        width = _grip_to_width(joint_state[7], self._closed_m, self._open_m)
        if self._last_grip_width is None or abs(width - self._last_grip_width) > self._grip_eps_m:
            try:
                self._grip_future = self._gripper.move_async(width, self._grip_speed)
                self._last_grip_width = width
            except Exception as e:
                print(f"\n[franky] gripper error: {e}")

    def get_observations(self) -> Dict[str, np.ndarray]:
        js = self._robot.current_joint_state
        q = np.asarray(js.position, dtype=float)[:7]
        try:
            dq = np.asarray(js.velocity, dtype=float)[:7]
        except Exception:
            dq = np.zeros(7)
        grip = _width_to_grip(self._gripper.width, self._closed_m, self._open_m)
        return {
            "joint_positions": np.concatenate([q, [grip]]),
            "joint_velocities": np.concatenate([dq, [0.0]]),
            "ee_pos_quat": self._read_ee_pose(),
            "gripper_position": np.array([grip]),
        }

    def _read_ee_pose(self) -> np.ndarray:
        try:
            pose = self._robot.current_cartesian_state.pose.end_effector_pose
            pos = np.asarray(pose.translation, dtype=float).reshape(3)
            quat = np.asarray(pose.quaternion, dtype=float).reshape(4)  # franky [x,y,z,w]
            return np.concatenate([pos, quat])
        except Exception:
            ident = np.zeros(7)
            ident[3] = 1.0
            return ident


def main() -> None:
    p = argparse.ArgumentParser(description="franky FR3 follower served over ZMQ.")
    p.add_argument("--arm", choices=list(C.ARMS), default="right")
    p.add_argument("--read-only", action="store_true",
                   help="step 2: serve state only, ignore commands (cannot move robot)")
    p.add_argument("--ip", default=None, help="override robot IP from config")
    p.add_argument("--port", type=int, default=None, help="override ZMQ port from config")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--relative-dynamics", type=float, default=0.08,
                   help="franky speed/accel/jerk scale in (0,1]; start low")
    args = p.parse_args()

    cfg = C.ARMS[args.arm]
    port = args.port if args.port is not None else cfg["robot_zmq_port"]
    follower = FrankyFollower(
        robot_ip=args.ip or cfg["robot_ip"],
        grip_width_rng=cfg["grip_width_rng"],
        read_only=args.read_only,
        relative_dynamics=args.relative_dynamics,
    )
    server = ZMQRobotServer(follower, host=args.host, port=port)
    mode = "read-only (step 2)" if args.read_only else "control (step 3)"
    print(f"follower 🚀  arm={args.arm} mode={mode} port={port}  (Ctrl-C to stop)")
    try:
        server.serve()
    except KeyboardInterrupt:
        print("\nstopped")
        server.stop()


if __name__ == "__main__":
    main()
