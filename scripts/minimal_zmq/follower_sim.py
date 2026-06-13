"""Step 1 follower: a MuJoCo dual-arm visualization that serves the follower
protocol over ZMQ.

    mjpython follower_sim.py        # macOS (viewer needs the mjpython launcher)
    python   follower_sim.py        # Linux

The teleop client (teleop.py) streams [q(7), grip(1)] for the controlled arm over
ZMQ; this node applies it as the MuJoCo control target and you watch the arm move.
This is the SAME teleop client and the SAME ZMQ protocol used against the real
franky follower in step 3 — so getting the arm moving here validates the whole
control path before any real hardware is involved.

Threading: MuJoCo's model/data and the viewer must be touched from ONE thread, so
mj_step + viewer.sync run on the main thread and the ZMQ server runs in a daemon
thread that only reads/writes plain numpy command/state buffers under a lock.

By default the right arm is the controlled, ZMQ-exposed 8-DOF follower; the left
arm just holds home as a backdrop. Use --arm to switch.
"""
import argparse
import platform
import sys
import threading
import time

import numpy as np

import config as C
from sim import DualArmSim
from zmq_transport import ZMQRobotServer, ZMQServerThread

NUM_DOFS = 8  # 7 arm joints + 1 gripper


def _check_mjpython() -> None:
    import os

    if platform.system() == "Darwin" and "MJPYTHON_BIN" not in os.environ:
        print(
            "ERROR: on macOS the viewer needs mjpython.\n"
            "  Run:  mjpython follower_sim.py\n"
            f"  (current interpreter: {sys.executable})"
        )
        sys.exit(1)


class SimFollower:
    """Follower protocol backed by one arm of the MuJoCo sim. The ZMQ thread calls
    these; they only touch the locked command/state buffers, never mujoco directly.
    The main loop (run_viewer) consumes the command buffer and refreshes state."""

    def __init__(self, controlled_arm: str):
        self._arm = controlled_arm
        self._lock = threading.Lock()
        self._cmd = np.array(C.FR3_HOME + [0.0], dtype=float)   # latest target
        self._state = np.array(C.FR3_HOME + [0.0], dtype=float)  # latest readback

    # ---- follower protocol (called from the ZMQ thread) ----
    def num_dofs(self) -> int:
        return NUM_DOFS

    def get_joint_state(self) -> np.ndarray:
        with self._lock:
            return self._state.copy()

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        with self._lock:
            self._cmd = np.asarray(joint_state, dtype=float)[:NUM_DOFS].copy()

    def get_observations(self):
        with self._lock:
            s = self._state.copy()
        ee = np.zeros(7)
        ee[3] = 1.0
        return {
            "joint_positions": s,
            "joint_velocities": np.zeros(NUM_DOFS),
            "ee_pos_quat": ee,
            "gripper_position": np.array([s[-1]]),
        }

    # ---- main-thread helpers ----
    def latest_cmd(self) -> np.ndarray:
        with self._lock:
            return self._cmd.copy()

    def set_state(self, state: np.ndarray) -> None:
        with self._lock:
            self._state = np.asarray(state, dtype=float).copy()


def run_viewer(follower: SimFollower, controlled_arm: str, host: str, port: int) -> None:
    """Live interactive window (needs mjpython on macOS)."""
    import mujoco
    import mujoco.viewer

    sim = DualArmSim()
    other_arm = "left" if controlled_arm == "right" else "right"
    home_cmd = np.array(C.FR3_HOME + [0.0])

    server = ZMQRobotServer(follower, host=host, port=port)
    server_thread = ZMQServerThread(server)
    server_thread.start()

    period = 1.0 / C.HZ
    n_steps = max(1, round(period / sim.model.opt.timestep))
    print(f"follower_sim 🚀  arm={controlled_arm} port={port}  (close viewer / Ctrl-C to stop)")
    try:
        with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
            while viewer.is_running():
                t0 = time.time()
                sim.set_command({
                    controlled_arm: follower.latest_cmd(),
                    other_arm: home_cmd,
                })
                for _ in range(n_steps):
                    mujoco.mj_step(sim.model, sim.data)
                follower.set_state(sim.read_arm(controlled_arm))
                viewer.sync()
                dt = period - (time.time() - t0)
                if dt > 0:
                    time.sleep(dt)
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        server_thread.terminate()


def run_record(follower: SimFollower, controlled_arm: str, host: str, port: int,
               out_path: str, seconds: float, fps: int = 30) -> None:
    """Headless offscreen render to an MP4 (no window — works on any Mac, and
    lets you review the teleop without a GUI). Steps the same control loop as the
    live viewer for `seconds`, writing one frame per render tick."""
    import imageio
    import mujoco

    sim = DualArmSim()
    other_arm = "left" if controlled_arm == "right" else "right"
    home_cmd = np.array(C.FR3_HOME + [0.0])

    server = ZMQRobotServer(follower, host=host, port=port)
    server_thread = ZMQServerThread(server)
    server_thread.start()

    renderer = mujoco.Renderer(sim.model, height=480, width=640)
    period = 1.0 / fps
    n_steps = max(1, round(period / sim.model.opt.timestep))
    n_frames = int(seconds * fps)
    print(f"follower_sim REC 🎥  arm={controlled_arm} port={port} -> {out_path} "
          f"({seconds}s @ {fps}fps)")
    writer = imageio.get_writer(out_path, fps=fps)
    try:
        for _ in range(n_frames):
            t0 = time.time()
            sim.set_command({
                controlled_arm: follower.latest_cmd(),
                other_arm: home_cmd,
            })
            for _ in range(n_steps):
                mujoco.mj_step(sim.model, sim.data)
            follower.set_state(sim.read_arm(controlled_arm))
            renderer.update_scene(sim.data)
            writer.append_data(renderer.render())
            dt = period - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        writer.close()
        renderer.close()
        server_thread.terminate()
        print(f"\nsaved {out_path}")


def main() -> None:
    p = argparse.ArgumentParser(description="Step 1: MuJoCo follower served over ZMQ.")
    p.add_argument("--arm", choices=list(C.ARMS), default="right",
                   help="which arm is the controlled, ZMQ-exposed follower")
    p.add_argument("--port", type=int, default=None, help="override ZMQ port from config")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--record", default=None, metavar="OUT.mp4",
                   help="headless: render to this MP4 instead of opening a window "
                        "(no mjpython needed; great for Mac/CI review)")
    p.add_argument("--seconds", type=float, default=10.0,
                   help="recording length when --record is set")
    p.add_argument("--fps", type=int, default=30, help="recording frame rate")
    args = p.parse_args()

    port = args.port if args.port is not None else C.ARMS[args.arm]["robot_zmq_port"]
    follower = SimFollower(args.arm)
    if args.record:
        run_record(follower, args.arm, args.host, port, args.record, args.seconds, args.fps)
    else:
        _check_mjpython()
        run_viewer(follower, args.arm, args.host, port)


if __name__ == "__main__":
    main()
