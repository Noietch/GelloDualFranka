"""Unified dual-arm teleop launcher (config-driven, sim or real).

Reads configs/bimanual.yaml. The `arms` section is the single source of truth
for each GELLO (port + calibration); `port: none` = shown-but-uncontrolled.
Switching Mac->Linux or sim->real is a config edit only (port strings / backend),
no code change.

Backends:
  sim : connects over ZMQ to the MuJoCo node
          mjpython experiments/launch_nodes.py --robot sim_bimanual_panda
  ros : real robot via ROS2 — NOT driven from here; the ROS2 stack consumes the
        same arm calibration. This launcher refuses `backend: ros` so you don't
        accidentally command a real arm through the sim path.

Usage:
    # terminal 1
    mjpython experiments/launch_nodes.py --robot sim_bimanual_panda
    # terminal 2
    python experiments/run_bimanual.py --config configs/bimanual.yaml
"""
import time
from dataclasses import dataclass

import numpy as np
import tyro
from omegaconf import OmegaConf

from gello.agents.agent import BimanualAgent, PassiveAgent
from gello.agents.gello_agent import DynamixelRobotConfig, GelloAgent
from gello.env import RobotEnv
from gello.zmq_core.robot_node import ZMQClientRobot


@dataclass
class Args:
    config: str = "configs/bimanual.yaml"
    robot_port: int = 6001
    hostname: str = "127.0.0.1"


# Panda home pose (qpos) — an uncontrolled arm holds this instead of collapsing.
# 7 arm joints + gripper (0 = open in the env's [0,1] gripper convention).
PANDA_HOME = [0.0, 0.0, 0.0, -1.57079, 0.0, 1.57079, -0.7853, 0.0]


def build_arm_agent(arm_cfg, n_dofs_each):
    """Return (agent, controlled) for one arm. port=none -> PassiveAgent."""
    port = arm_cfg.get("port", "none")
    if port is None or str(port).lower() == "none":
        hold = arm_cfg.get("hold_pose", None) or PANDA_HOME
        return PassiveAgent(num_dofs=n_dofs_each, hold_pose=hold[:n_dofs_each]), False
    dxl = DynamixelRobotConfig(
        joint_ids=tuple(arm_cfg["joint_ids"]),
        joint_offsets=tuple(arm_cfg["joint_offsets"]),
        joint_signs=tuple(arm_cfg["joint_signs"]),
        gripper_config=tuple(arm_cfg["gripper_config"]),
    )
    start = arm_cfg.get("start_joints", None)
    start = np.array(start) if start is not None else None
    return GelloAgent(port=str(port), dynamixel_config=dxl, start_joints=start), True


def main(args: Args):
    cfg = OmegaConf.to_container(OmegaConf.load(args.config), resolve=True)

    backend = cfg.get("backend", "sim")
    if backend != "sim":
        raise SystemExit(
            f"backend='{backend}' is not driven by this launcher. Real-robot "
            f"control runs through the ROS2 stack; this script only drives the "
            f"MuJoCo sim. Set backend: sim to use it."
        )

    robot_client = ZMQClientRobot(port=args.robot_port, host=args.hostname)
    env = RobotEnv(robot_client, control_rate_hz=cfg.get("hz", 60), camera_dict={})

    total = robot_client.num_dofs()
    n_each = total // 2
    assert n_each * 2 == total, f"sim dofs {total} not even"

    left_agent, lc = build_arm_agent(cfg["arms"]["left"], n_each)
    right_agent, rc = build_arm_agent(cfg["arms"]["right"], n_each)
    agent = BimanualAgent(left_agent, right_agent)
    print(f"left {'CONTROLLED' if lc else 'passive'}, "
          f"right {'CONTROLLED' if rc else 'passive'}, per-arm dofs={n_each}")

    # Ease the sim to the agents' first target so a controlled arm doesn't jump;
    # passive arms target their own current pose and stay put.
    obs = env.get_obs()
    start_pos = agent.act(obs)
    joints = obs["joint_positions"]
    abs_deltas = np.abs(start_pos - joints)
    steps = min(int(abs_deltas.max() / 0.01), 150) if abs_deltas.max() > 0 else 1
    for jnt in np.linspace(joints, start_pos, max(steps, 1)):
        env.step(jnt)
        time.sleep(0.002)

    print("Start 🚀  (Ctrl-C to stop)")
    max_delta = 0.2
    try:
        while True:
            obs = env.get_obs()
            cmd = agent.act(obs)
            cur = obs["joint_positions"]
            delta = cmd - cur
            for s in (slice(0, n_each), slice(n_each, 2 * n_each)):
                m = np.abs(delta[s]).max()
                if m > max_delta:
                    delta[s] = delta[s] / m * max_delta
            env.step(cur + delta)
    except KeyboardInterrupt:
        print("\nstopped")


if __name__ == "__main__":
    main(tyro.cli(Args))
