"""Minimal GELLO CLI — three modes, one entry point. No gello package, no ZMQ.

    # Mac or Linux, no robot needed:
    mjpython main.py teleop-sim     # GELLO -> MuJoCo dual-arm sim
    #   (on Linux: python main.py teleop-sim)

    # Linux + franka_ros2, real robot:
    python main.py sync-robot       # real robot ROS2 state -> MuJoCo (mirror)
    python main.py teleop-robot     # GELLO -> gello/joint_states for the C++ controller

rclpy is imported lazily inside the ROS modes only, so teleop-sim runs on a Mac
that has no rclpy. Edit hardware config at the top of gello_mujoco.py.
"""
import argparse
import platform
import sys
import time
from typing import Optional

import numpy as np

import gello_mujoco as G


# ------------------------------ helpers ------------------------------

def _open_readers():
    """Open a GelloReader per controlled arm. Returns {name: reader|None}."""
    ports = G.current_ports()
    readers = {}
    for name, cfg in G.ARMS.items():
        port = ports.get(name, "none")
        if port is None or str(port).lower() == "none":
            readers[name] = None
            print(f"[{name}] port=none -> uncontrolled (holds home)")
            continue
        ids = list(cfg["joint_ids"]) + [cfg["gripper_id"]]
        print(f"[{name}] opening {port} (ids {ids})")
        readers[name] = G.GelloReader(ids, port)
    return readers


def _check_mjpython():
    """On macOS, mujoco.viewer requires the mjpython launcher. mjpython re-execs
    into the normal python (so sys.executable looks like plain python3); the
    reliable signal is the MJPYTHON_BIN env var it sets."""
    import os

    if platform.system() == "Darwin" and "MJPYTHON_BIN" not in os.environ:
        print(
            "ERROR: on macOS the viewer needs mjpython.\n"
            "  Run:  mjpython main.py <mode>\n"
            f"  (current interpreter: {sys.executable})"
        )
        sys.exit(1)


# ------------------------------ mode: teleop-sim ------------------------------

def teleop_sim() -> None:
    """GELLO -> MuJoCo dual-arm. Pure Python; no ROS2, no real robot."""
    _check_mjpython()
    import mujoco
    import mujoco.viewer

    sim = G.DualArmSim()
    readers = _open_readers()

    def read_arm_ctrl(name) -> np.ndarray:
        """Return this arm's 8-vec [j1..j7, grip(0-1)] for data.ctrl."""
        cfg = G.ARMS[name]
        reader = readers[name]
        if reader is None:
            return np.array(G.FR3_HOME + [0.0])
        raw = reader.get_joints()
        arm = G.calib_sim(raw[:7], cfg["sim_offsets"], cfg["sim_signs"])
        grip = G.map_gripper_deg(raw[-1], cfg["gripper_deg"][0], cfg["gripper_deg"][1])
        return np.concatenate([arm, [grip]])

    print("teleop-sim 🚀  (close viewer or Ctrl-C to stop)")
    period = 1.0 / G.HZ_SIM
    # panda timestep is ~2ms; one mj_step per 60Hz frame would run sim at ~0.12x
    # real time (the arm lags badly). Step enough times per frame to cover the
    # wall-clock frame period so the PD-driven arm tracks in real time.
    n_steps = max(1, round(period / sim.model.opt.timestep))
    try:
        with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
            while viewer.is_running():
                t0 = time.time()
                sim.set_command({
                    "left": read_arm_ctrl("left"),
                    "right": read_arm_ctrl("right"),
                })
                for _ in range(n_steps):
                    mujoco.mj_step(sim.model, sim.data)
                viewer.sync()
                dt = period - (time.time() - t0)
                if dt > 0:
                    time.sleep(dt)
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        for r in readers.values():
            if r is not None:
                r.close()


# ------------------------------ mode: sync-robot ------------------------------

def sync_robot() -> None:
    """Real robot ROS2 joint_states -> MuJoCo (one-way mirror, no GELLO read)."""
    _check_mjpython()
    import mujoco
    import mujoco.viewer
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    sim = G.DualArmSim()
    latest = {"left": None, "right": None}

    def reorder(msg) -> Optional[list]:
        """Pull fr3_joint1..7 out of a JointState BY NAME. joint_state_broadcaster
        does not guarantee position[] is in fr3_joint1..7 order — indexing
        position[:7] blindly scrambles the arm. Map name->value instead."""
        by_name = dict(zip(msg.name, msg.position))
        try:
            return [by_name[j] for j in G.FR3_JOINT_NAMES]
        except KeyError:
            return None  # arm joints not present yet (e.g. gripper-only msg)

    class Mirror(Node):
        def __init__(self):
            super().__init__("gello_sync_robot")
            for name in G.ARMS:
                topic = G.ROBOT_STATE_TOPIC.format(ns=name)
                self.create_subscription(
                    JointState, topic,
                    lambda msg, n=name: latest.__setitem__(n, reorder(msg)),
                    10,
                )
                self.get_logger().info(f"subscribing {topic}")

    rclpy.init()
    node = Mirror()
    print("sync-robot 🚀  (close viewer or Ctrl-C to stop)")
    try:
        with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
            while viewer.is_running():
                rclpy.spin_once(node, timeout_sec=0.0)
                for name in G.ARMS:
                    q = latest[name]
                    if q is not None and len(q) >= 7:
                        sim.write_arm_qpos(name, np.array(q[:7]))
                mujoco.mj_forward(sim.model, sim.data)
                viewer.sync()
                time.sleep(1.0 / G.HZ_SIM)
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


# ------------------------------ mode: teleop-robot ------------------------------

def teleop_robot() -> None:
    """GELLO -> ROS2: publish gello/joint_states (+ gripper) per arm for the
    official C++ joint-impedance controller. Needs system ROS2 + franka_ros2."""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float32

    readers = _open_readers()

    class Publisher(Node):
        def __init__(self):
            super().__init__("gello_teleop_robot")
            self.arm_pub, self.grip_pub = {}, {}
            for name in G.ARMS:
                if readers[name] is None:
                    continue
                self.arm_pub[name] = self.create_publisher(
                    JointState, G.GELLO_PUB_TOPIC.format(ns=name), 10)
                self.grip_pub[name] = self.create_publisher(
                    Float32, G.GRIPPER_TOPIC.format(ns=name), 10)
                self.get_logger().info(f"publishing {G.GELLO_PUB_TOPIC.format(ns=name)}")
            self.timer = self.create_timer(1.0 / G.HZ_ROS, self.tick)

        def tick(self):
            for name, cfg in G.ARMS.items():
                if readers[name] is None:
                    continue
                raw = readers[name].get_joints()
                arm = G.calib_real(raw[:7], cfg["real_offsets"], cfg["real_signs"])
                grip = G.map_gripper_rad(
                    raw[-1], cfg["gripper_range_rad"][0], cfg["gripper_range_rad"][1])

                js = JointState()
                js.header.stamp = self.get_clock().now().to_msg()
                js.header.frame_id = "fr3_link0"
                js.name = G.FR3_JOINT_NAMES
                js.position = arm.tolist()
                self.arm_pub[name].publish(js)

                g = Float32()
                g.data = float(grip)
                self.grip_pub[name].publish(g)

    rclpy.init()
    node = Publisher()
    print("teleop-robot 🚀  (Ctrl-C to stop)")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        for r in readers.values():
            if r is not None:
                r.close()


# ------------------------------ CLI ------------------------------

_MODES = {
    "teleop-sim": teleop_sim,      # GELLO -> MuJoCo sim (no ROS2)
    "sync-robot": sync_robot,      # real robot ROS2 state -> MuJoCo mirror
    "teleop-robot": teleop_robot,  # GELLO -> gello/joint_states for controller
}


def main():
    parser = argparse.ArgumentParser(
        description="Minimal GELLO CLI (teleop-sim / sync-robot / teleop-robot).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("mode", choices=list(_MODES), help="which mode to run")
    args = parser.parse_args()
    _MODES[args.mode]()


if __name__ == "__main__":
    main()
