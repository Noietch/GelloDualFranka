"""Step 2 client: mirror the REAL robot's state into MuJoCo (visualization only).

    mjpython mirror.py --host <robot_pc_ip>     # macOS
    python   mirror.py --host <robot_pc_ip>     # Linux

Connects to the franky follower running with --read-only on the robot PC, pulls
its joint state over ZMQ, and kinematically writes it into the MuJoCo arm (no
physics step — a pure mirror). This sends NO commands, so it is the safe way to
confirm the franky connection and that the real joint angles line up with the sim
before any teleop. Mirrors the controlled arm; the other arm holds home.
"""
import argparse
import platform
import sys
import time

import numpy as np

import config as C
from sim import DualArmSim
from zmq_transport import ZMQRobotClient


def _check_mjpython() -> None:
    import os

    if platform.system() == "Darwin" and "MJPYTHON_BIN" not in os.environ:
        print(
            "ERROR: on macOS the viewer needs mjpython.\n"
            "  Run:  mjpython mirror.py --host <robot_pc_ip>\n"
            f"  (current interpreter: {sys.executable})"
        )
        sys.exit(1)


def main() -> None:
    p = argparse.ArgumentParser(description="Step 2: mirror real robot state into MuJoCo.")
    p.add_argument("--arm", choices=list(C.ARMS), default="right")
    p.add_argument("--host", default="127.0.0.1", help="robot PC IP running follower.py")
    p.add_argument("--port", type=int, default=None, help="override ZMQ port from config")
    args = p.parse_args()

    _check_mjpython()
    import mujoco
    import mujoco.viewer

    port = args.port if args.port is not None else C.ARMS[args.arm]["robot_zmq_port"]
    robot = ZMQRobotClient(port=port, host=args.host)
    print(f"[mirror] connected follower on tcp://{args.host}:{port} (dofs={robot.num_dofs()})")

    sim = DualArmSim()
    other_arm = "left" if args.arm == "right" else "right"
    sim.write_arm_qpos(other_arm, np.array(C.FR3_HOME))

    print(f"mirror 🚀  arm={args.arm}  (close viewer / Ctrl-C to stop)")
    try:
        with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
            while viewer.is_running():
                state = robot.get_joint_state()       # [q(7), grip(1)]
                sim.write_arm_qpos(args.arm, state[:7])
                mujoco.mj_forward(sim.model, sim.data)
                viewer.sync()
                time.sleep(1.0 / C.HZ)
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        robot.close()


if __name__ == "__main__":
    main()
