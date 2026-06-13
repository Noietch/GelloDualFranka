import pickle
import threading
import time
from typing import Any, Dict, Optional

import mujoco
import mujoco.viewer
import numpy as np
import zmq
from dm_control import mjcf

from gello.robots.robot import Robot

assert mujoco.viewer is mujoco.viewer


def attach_hand_to_arm(
    arm_mjcf: mjcf.RootElement,
    hand_mjcf: mjcf.RootElement,
) -> None:
    """Attaches a hand to an arm.

    The arm must have a site named "attachment_site".

    Taken from https://github.com/deepmind/mujoco_menagerie/blob/main/FAQ.md#how-do-i-attach-a-hand-to-an-arm

    Args:
      arm_mjcf: The mjcf.RootElement of the arm.
      hand_mjcf: The mjcf.RootElement of the hand.

    Raises:
      ValueError: If the arm does not have a site named "attachment_site".
    """
    physics = mjcf.Physics.from_mjcf_model(hand_mjcf)

    attachment_site = arm_mjcf.find("site", "attachment_site")
    if attachment_site is None:
        raise ValueError("No attachment site found in the arm model.")

    # Expand the ctrl and qpos keyframes to account for the new hand DoFs.
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


def _add_floor_and_light(arena):
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
    arena.worldbody.add(
        "light", pos=[0, 0, 1.5], dir=[0, 0, -1], directional=True,
    )


def _attach_arm(arena, robot_xml_path, gripper_xml_path=None, base_quat=None,
                base_pos=None, name=None):
    arm = mjcf.from_path(robot_xml_path)
    if name is not None:
        arm.model = name  # namespace so dual arms don't collide on element names
    if gripper_xml_path is not None:
        attach_hand_to_arm(arm, mjcf.from_path(gripper_xml_path))
    frame = arena.worldbody.attach(arm)
    if base_quat is not None:
        frame.quat = list(base_quat)
    if base_pos is not None:
        frame.pos = list(base_pos)
    return arm


def build_scene(robot_xml_path: str, gripper_xml_path: Optional[str] = None,
                base_quat=None, base_pos=None, arms=None):
    """Build a MuJoCo scene.

    Single arm (back-compat): pass robot_xml_path (+ optional base_quat/pos).
    Multi arm: pass `arms` = list of dicts with keys
        {xml, gripper (opt), quat (opt), pos (opt), name (opt)}.
    """
    arena = mjcf.RootElement()
    _add_floor_and_light(arena)

    if arms is None:
        arms = [{
            "xml": robot_xml_path, "gripper": gripper_xml_path,
            "quat": base_quat, "pos": base_pos, "name": None,
        }]
    for spec in arms:
        _attach_arm(
            arena, spec["xml"], spec.get("gripper"),
            base_quat=spec.get("quat"), base_pos=spec.get("pos"),
            name=spec.get("name"),
        )

    return arena


class ZMQServerThread(threading.Thread):
    def __init__(self, server):
        super().__init__()
        self._server = server

    def run(self):
        self._server.serve()

    def terminate(self):
        self._server.stop()


class ZMQRobotServer:
    """A class representing a ZMQ server for a robot."""

    def __init__(self, robot: Robot, host: str = "127.0.0.1", port: int = 5556):
        self._robot = robot
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.REP)
        addr = f"tcp://{host}:{port}"
        self._socket.bind(addr)
        self._stop_event = threading.Event()

    def serve(self) -> None:
        """Serve the robot state and commands over ZMQ."""
        self._socket.setsockopt(zmq.RCVTIMEO, 1000)  # Set timeout to 1000 ms
        while not self._stop_event.is_set():
            try:
                message = self._socket.recv()
                request = pickle.loads(message)

                # Call the appropriate method based on the request
                method = request.get("method")
                args = request.get("args", {})
                result: Any
                if method == "num_dofs":
                    result = self._robot.num_dofs()
                elif method == "get_joint_state":
                    result = self._robot.get_joint_state()
                elif method == "command_joint_state":
                    result = self._robot.command_joint_state(**args)
                elif method == "get_observations":
                    result = self._robot.get_observations()
                else:
                    result = {"error": "Invalid method"}
                    print(result)
                    raise NotImplementedError(
                        f"Invalid method: {method}, {args, result}"
                    )

                self._socket.send(pickle.dumps(result))
            except zmq.error.Again:
                print("Timeout in ZMQLeaderServer serve")
                # Timeout occurred, check if the stop event is set

    def stop(self) -> None:
        self._stop_event.set()
        self._socket.close()
        self._context.term()


class MujocoRobotServer:
    def __init__(
        self,
        xml_path: str = None,
        gripper_xml_path: Optional[str] = None,
        host: str = "127.0.0.1",
        port: int = 5556,
        print_joints: bool = False,
        base_quat=None,
        base_pos=None,
        arms=None,
    ):
        self._has_gripper = gripper_xml_path is not None
        arena = build_scene(xml_path, gripper_xml_path, base_quat=base_quat,
                            base_pos=base_pos, arms=arms)

        assets: Dict[str, str] = {}
        for asset in arena.asset.all_children():
            if asset.tag == "mesh":
                f = asset.file
                assets[f.get_vfs_filename()] = asset.file.contents

        xml_string = arena.to_xml_string()
        # save xml_string to file
        with open("arena.xml", "w") as f:
            f.write(xml_string)

        self._model = mujoco.MjModel.from_xml_string(xml_string, assets)
        self._data = mujoco.MjData(self._model)

        self._num_joints = self._model.nu

        # Map each actuator (command index) to the qpos/qvel address it reads
        # back from, so observations line up 1:1 with commands. Arm actuators
        # drive a joint directly; the gripper actuator drives a TENDON (two
        # finger joints), proxied here by finger_joint1 in the same namespace.
        # Without this, a dual-arm assembly — each panda adds 2 finger qpos but
        # only 1 gripper actuator — makes a naive qpos[:nu] slice misaligned, so
        # the right arm reads the wrong joints (off by the left arm's extra
        # finger_joint2) and teleop maps garbage.
        qadr, dadr, grip_ctrl = [], [], []
        for i in range(self._model.nu):
            aname = mujoco.mj_id2name(
                self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, i
            ) or ""
            if self._model.actuator_trntype[i] == mujoco.mjtTrn.mjTRN_JOINT:
                jid = self._model.actuator_trnid[i, 0]
            else:  # tendon -> gripper; read back the arm's finger_joint1
                grip_ctrl.append(i)
                prefix = aname.rsplit("/", 1)[0] + "/" if "/" in aname else ""
                jid = mujoco.mj_name2id(
                    self._model, mujoco.mjtObj.mjOBJ_JOINT, prefix + "finger_joint1"
                )
                if jid < 0:
                    jid = 0
            qadr.append(self._model.jnt_qposadr[jid])
            dadr.append(self._model.jnt_dofadr[jid])
        self._obs_qadr = np.array(qadr, dtype=int)
        self._obs_dadr = np.array(dadr, dtype=int)
        self._gripper_ctrl_idx = np.array(grip_ctrl, dtype=int)

        self._joint_state = np.zeros(self._num_joints)
        self._joint_cmd = self._joint_state

        self._zmq_server = ZMQRobotServer(robot=self, host=host, port=port)
        self._zmq_server_thread = ZMQServerThread(self._zmq_server)

        self._print_joints = print_joints

    def num_dofs(self) -> int:
        return self._num_joints

    def get_joint_state(self) -> np.ndarray:
        return self._joint_state

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        assert len(joint_state) == self._num_joints, (
            f"Expected joint state of length {self._num_joints}, "
            f"got {len(joint_state)}."
        )
        cmd = np.asarray(joint_state, dtype=float).copy()
        # Gripper actuator ctrlrange is [0,255]; GELLO sends [0,1]. Scale every
        # gripper actuator (one per arm), not just the last index — the last
        # index is the right arm's gripper, leaving the left arm's unscaled.
        if self._gripper_ctrl_idx.size:
            cmd[self._gripper_ctrl_idx] = cmd[self._gripper_ctrl_idx] * 255
        self._joint_cmd = cmd

    def freedrive_enabled(self) -> bool:
        return True

    def set_freedrive_mode(self, enable: bool):
        pass

    def get_observations(self) -> Dict[str, np.ndarray]:
        # Index by the per-actuator qpos/qvel map so obs[i] is the joint that
        # command[i] drives. A flat qpos[:nu] slice misaligns the right arm in a
        # dual-arm scene (each panda inserts 2 finger qpos but exposes 1 gripper
        # actuator), which is what made the right arm read the wrong joints.
        joint_positions = self._data.qpos.copy()[self._obs_qadr]
        joint_velocities = self._data.qvel.copy()[self._obs_dadr]
        # Gripper qpos is finger travel in meters (~0..0.04); report it back in
        # the [0,1] convention the env uses.
        if self._gripper_ctrl_idx.size:
            joint_positions[self._gripper_ctrl_idx] = np.clip(
                joint_positions[self._gripper_ctrl_idx] / 0.04, 0.0, 1.0
            )
        ee_site = "attachment_site"
        try:
            ee_pos = self._data.site_xpos.copy()[
                mujoco.mj_name2id(self._model, 6, ee_site)
            ]
            ee_mat = self._data.site_xmat.copy()[
                mujoco.mj_name2id(self._model, 6, ee_site)
            ]
            ee_quat = np.zeros(4)
            mujoco.mju_mat2Quat(ee_quat, ee_mat)
        except Exception:
            ee_pos = np.zeros(3)
            ee_quat = np.zeros(4)
            ee_quat[0] = 1
        gripper_pos = joint_positions[-1]
        return {
            "joint_positions": joint_positions,
            "joint_velocities": joint_velocities,
            "ee_pos_quat": np.concatenate([ee_pos, ee_quat]),
            "gripper_position": gripper_pos,
        }

    def serve(self) -> None:
        # start the zmq server
        self._zmq_server_thread.start()
        with mujoco.viewer.launch_passive(self._model, self._data) as viewer:
            while viewer.is_running():
                step_start = time.time()

                # mj_step can be replaced with code that also evaluates
                # a policy and applies a control signal before stepping the physics.
                self._data.ctrl[:] = self._joint_cmd
                # self._data.qpos[:] = self._joint_cmd
                mujoco.mj_step(self._model, self._data)
                self._joint_state = self._data.qpos.copy()[self._obs_qadr]

                if self._print_joints:
                    print(self._joint_state)

                # Example modification of a viewer option: toggle contact points every two seconds.
                with viewer.lock():
                    # TODO remove?
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(
                        self._data.time % 2
                    )

                # Pick up changes to the physics state, apply perturbations, update options from GUI.
                viewer.sync()

                # Rudimentary time keeping, will drift relative to wall clock.
                time_until_next_step = self._model.opt.timestep - (
                    time.time() - step_start
                )
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

    def stop(self) -> None:
        self._zmq_server_thread.join()

    def __del__(self) -> None:
        self.stop()
