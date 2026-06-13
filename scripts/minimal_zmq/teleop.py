"""GELLO -> ZMQ teleop client. Used UNCHANGED for both:
  step 1: against follower_sim.py  (MuJoCo)   --calib sim
  step 3: against follower.py      (franky)   --calib real

Reads the GELLO arm, calibrates to FR3 joint targets, and streams [q(7), grip(1)]
to whichever follower is bound on the ZMQ port. The follower on the other end is
the only thing that changes between sim and real — this client does not know or
care which it is talking to.

Calibration MUST match the target (see gello_driver / config):
  --calib sim  : modulo-wrap         (step 1, into MuJoCo)
  --calib real : incremental + clip  (step 3, into franky)
Using 'real' against sim or vice versa drives joints wrong (the joint4-frozen
symptom on the real robot). The step scripts pass the right one for you.

Startup safety: before streaming, compare the leader's first target to the
follower's current pose; if any joint is more than --abort-delta rad apart, abort
(so the robot won't jump). Otherwise ramp to the start pose in small steps.

--leader dummy gives a hardware-free sine (no GELLO needed) to smoke-test the link.
"""
import argparse
import time

import numpy as np

import config as C
import gello_driver as D
from zmq_transport import ZMQRobotClient


class GelloLeader:
    """Reads one GELLO arm and maps raw -> [q(7), grip(1)] with the chosen calib."""

    def __init__(self, arm: str, calib: str):
        cfg = C.ARMS[arm]
        self._reader = D.open_gello(arm)
        self._calib = calib
        self._cfg = cfg
        if calib == "real":
            self._inc = D.IncrementalCalibrator(cfg["real_offsets"], cfg["real_signs"])

    def act(self) -> np.ndarray:
        raw = self._reader.get_joints()
        cfg = self._cfg
        if self._calib == "sim":
            arm = D.calib_sim(raw[:7], cfg["sim_offsets"], cfg["sim_signs"])
            grip = D.map_gripper_deg(raw[-1], cfg["gripper_deg"][0], cfg["gripper_deg"][1])
        else:
            arm = self._inc.process(raw[:7])
            grip = D.map_gripper_rad(
                raw[-1], cfg["gripper_range_rad"][0], cfg["gripper_range_rad"][1])
        return np.concatenate([arm, [grip]])

    def close(self) -> None:
        self._reader.close()


class DummyLeader:
    """Hardware-free leader: gentle sine around the follower's start pose. Only for
    smoke-testing the link (never against a real robot)."""

    def __init__(self, start_pose: np.ndarray):
        self._base = np.asarray(start_pose, dtype=float).copy()
        self._t0 = time.time()

    def act(self) -> np.ndarray:
        cmd = self._base.copy()
        cmd[0] = self._base[0] + 0.3 * np.sin(0.5 * (time.time() - self._t0))
        return cmd

    def close(self) -> None:
        pass


def move_to_start(robot, leader, max_delta: float = 0.05, abort_delta: float = 0.8,
                  steps: int = 50) -> bool:
    """Ramp the follower to the leader's pose; abort if the initial gap is too big."""
    start = leader.act()
    cur = robot.get_joint_state()
    gap = np.abs(start - cur)
    if gap.max() > abort_delta:
        print("\nABORT: leader/follower too far apart on these joints:")
        for i in np.where(gap > abort_delta)[0]:
            print(f"  joint[{i}]: delta={gap[i]:.3f}  leader={start[i]:.3f}  follower={cur[i]:.3f}")
        print("Move the GELLO closer to the follower's current pose and retry.")
        return False
    print(f"Moving to start position ({steps} steps)...")
    for _ in range(steps):
        cur = robot.get_joint_state()
        delta = leader.act() - cur
        m = np.abs(delta).max()
        if m > max_delta:
            delta = delta / m * max_delta
        robot.command_joint_state(cur + delta)
        time.sleep(1.0 / C.HZ)
    return True


def main() -> None:
    p = argparse.ArgumentParser(description="GELLO -> ZMQ teleop (sim step 1 / real step 3).")
    p.add_argument("--arm", choices=list(C.ARMS), default="right")
    p.add_argument("--calib", choices=["sim", "real"], default="sim",
                   help="sim=modulo-wrap (step 1); real=incremental+clip (step 3)")
    p.add_argument("--leader", choices=["gello", "dummy"], default="gello")
    p.add_argument("--port", type=int, default=None, help="override ZMQ port from config")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--hz", type=int, default=C.HZ)
    args = p.parse_args()

    port = args.port if args.port is not None else C.ARMS[args.arm]["robot_zmq_port"]
    robot = ZMQRobotClient(port=port, host=args.host)
    print(f"[teleop] connected follower on tcp://{args.host}:{port} "
          f"(dofs={robot.num_dofs()}, calib={args.calib})")

    if args.leader == "gello":
        leader = GelloLeader(args.arm, args.calib)
    else:
        leader = DummyLeader(robot.get_joint_state())

    period = 1.0 / args.hz
    try:
        if not move_to_start(robot, leader):
            return
        print("\nTeleop 🚀🚀🚀  (Ctrl-C to stop)")
        start_time = time.time()
        while True:
            t0 = time.time()
            robot.command_joint_state(leader.act())
            print(f"\rTime: {time.time() - start_time:7.2f}s", end="", flush=True)
            dt = period - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        leader.close()
        robot.close()


if __name__ == "__main__":
    main()
