from dataclasses import dataclass
from pathlib import Path

import tyro

from gello.robots.robot import BimanualRobot, PrintRobot
from gello.zmq_core.robot_node import ZMQServerRobot


@dataclass
class Args:
    robot: str = "xarm"
    robot_port: int = 6001
    hostname: str = "127.0.0.1"
    robot_ip: str = "192.168.1.10"
    sim_arm: str = "none"
    """For sim robots: place the base at one arm of the real dual-arm rig.
    'left', 'right', or 'none' (upright). Transforms are the real install
    (right is the mirror of the measured left install)."""


# Real dual-arm base install transforms, from the rig's kinematics.yaml
# (urdf/kinematics.yaml: left/right base pose as xyz + rpy). Converted to
# MuJoCo quat (wxyz) here. These are the authoritative measured installs.
SIM_ARM_BASE = {
    "left": {
        "quat": [0.865807, -0.436878, 0.022288, -0.242939],
        "pos": [0.036395, 0.050681, 0.0508858],
    },
    "right": {
        "quat": [0.865807, 0.436878, 0.022288, 0.242939],
        "pos": [0.036395, -0.050681, 0.0508858],
    },
}


def launch_robot_server(args: Args):
    port = args.robot_port
    if args.robot == "sim_ur":
        MENAGERIE_ROOT: Path = (
            Path(__file__).parent.parent / "third_party" / "mujoco_menagerie"
        )
        xml = MENAGERIE_ROOT / "universal_robots_ur5e" / "ur5e.xml"
        gripper_xml = MENAGERIE_ROOT / "robotiq_2f85" / "2f85.xml"
        from gello.robots.sim_robot import MujocoRobotServer

        server = MujocoRobotServer(
            xml_path=xml, gripper_xml_path=gripper_xml, port=port, host=args.hostname
        )
        server.serve()
    elif args.robot == "sim_panda":
        from gello.robots.sim_robot import MujocoRobotServer

        MENAGERIE_ROOT: Path = (
            Path(__file__).parent.parent / "third_party" / "mujoco_menagerie"
        )
        xml = MENAGERIE_ROOT / "franka_emika_panda" / "panda.xml"
        gripper_xml = None
        base = SIM_ARM_BASE.get(args.sim_arm, {})
        server = MujocoRobotServer(
            xml_path=xml, gripper_xml_path=gripper_xml, port=port, host=args.hostname,
            base_quat=base.get("quat"), base_pos=base.get("pos")
        )
        server.serve()
    elif args.robot == "sim_bimanual_panda":
        from gello.robots.sim_robot import MujocoRobotServer

        MENAGERIE_ROOT: Path = (
            Path(__file__).parent.parent / "third_party" / "mujoco_menagerie"
        )
        xml = str(MENAGERIE_ROOT / "franka_emika_panda" / "panda.xml")
        # Both arms at their real install transforms (runtime-assembled, no
        # separate static xml). Joint order: [left 7 arm + gripper, right ...].
        arms = [
            {"xml": xml, "quat": SIM_ARM_BASE["left"]["quat"],
             "pos": SIM_ARM_BASE["left"]["pos"], "name": "left"},
            {"xml": xml, "quat": SIM_ARM_BASE["right"]["quat"],
             "pos": SIM_ARM_BASE["right"]["pos"], "name": "right"},
        ]
        server = MujocoRobotServer(arms=arms, port=port, host=args.hostname)
        server.serve()
    elif args.robot == "sim_xarm":
        from gello.robots.sim_robot import MujocoRobotServer

        MENAGERIE_ROOT: Path = (
            Path(__file__).parent.parent / "third_party" / "mujoco_menagerie"
        )
        xml = MENAGERIE_ROOT / "ufactory_xarm7" / "xarm7.xml"
        gripper_xml = None
        server = MujocoRobotServer(
            xml_path=xml, gripper_xml_path=gripper_xml, port=port, host=args.hostname
        )
        server.serve()

    else:
        if args.robot == "xarm":
            from gello.robots.xarm_robot import XArmRobot

            robot = XArmRobot(ip=args.robot_ip)
        elif args.robot == "ur":
            from gello.robots.ur import URRobot

            robot = URRobot(robot_ip=args.robot_ip)
        elif args.robot == "panda":
            from gello.robots.panda import PandaRobot

            robot = PandaRobot(robot_ip=args.robot_ip)
        elif args.robot == "bimanual_ur":
            from gello.robots.ur import URRobot

            # IP for the bimanual robot setup is hardcoded
            _robot_l = URRobot(robot_ip="192.168.2.10")
            _robot_r = URRobot(robot_ip="192.168.1.10")
            robot = BimanualRobot(_robot_l, _robot_r)
        elif args.robot == "none" or args.robot == "print":
            robot = PrintRobot(8)

        else:
            raise NotImplementedError(
                f"Robot {args.robot} not implemented, choose one of: sim_ur, xarm, ur, bimanual_ur, none"
            )
        server = ZMQServerRobot(robot, port=port, host=args.hostname)
        print(f"Starting robot server on port {port}")
        server.serve()


def main(args):
    launch_robot_server(args)


if __name__ == "__main__":
    main(tyro.cli(Args))
