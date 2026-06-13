"""Minimal ZMQ transport for the follower protocol.

A follower object implements:
    num_dofs() -> int
    get_joint_state() -> np.ndarray            # [q(7), grip(1)]
    command_joint_state(joint_state) -> None   # [q(7), grip(1)], 0=open..1=closed
    get_observations() -> Dict[str, np.ndarray]

ZMQRobotServer exposes it over ZMQ REP; ZMQRobotClient proxies it over REQ, so
the teleop client never imports mujoco/franky — it talks to whichever follower is
bound on the port. REP/REQ + pickle, mirroring gello/zmq_core/robot_node.py.
"""
import pickle
import threading
from typing import Any, Dict, Optional

import numpy as np
import zmq


class ZMQRobotServer:
    def __init__(self, robot, host: str = "127.0.0.1", port: int = 6001):
        self._robot = robot
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.REP)
        addr = f"tcp://{host}:{port}"
        print(f"[robot-server] binding {addr}  ({robot})")
        self._socket.bind(addr)
        self._stop_event = threading.Event()

    def serve(self) -> None:
        self._socket.setsockopt(zmq.RCVTIMEO, 1000)  # 1s so stop() is responsive
        while not self._stop_event.is_set():
            try:
                request = pickle.loads(self._socket.recv())
                method = request.get("method")
                args = request.get("args", {})
                if method == "num_dofs":
                    result: Any = self._robot.num_dofs()
                elif method == "get_joint_state":
                    result = self._robot.get_joint_state()
                elif method == "command_joint_state":
                    result = self._robot.command_joint_state(**args)
                elif method == "get_observations":
                    result = self._robot.get_observations()
                else:
                    result = {"error": f"Invalid method {method}"}
                self._socket.send(pickle.dumps(result))
            except zmq.Again:
                pass  # recv timeout -> re-check stop flag

    def stop(self) -> None:
        self._stop_event.set()


class ZMQRobotClient:
    def __init__(self, port: int = 6001, host: str = "127.0.0.1"):
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.REQ)
        self._socket.connect(f"tcp://{host}:{port}")

    def _call(self, method: str, args: Optional[dict] = None) -> Any:
        self._socket.send(pickle.dumps({"method": method, "args": args or {}}))
        result = pickle.loads(self._socket.recv())
        if isinstance(result, dict) and "error" in result:
            raise RuntimeError(result["error"])
        return result

    def num_dofs(self) -> int:
        return self._call("num_dofs")

    def get_joint_state(self) -> np.ndarray:
        return self._call("get_joint_state")

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        return self._call("command_joint_state", {"joint_state": joint_state})

    def get_observations(self) -> Dict[str, np.ndarray]:
        return self._call("get_observations")

    def close(self) -> None:
        self._socket.close()
        self._context.term()


class ZMQServerThread(threading.Thread):
    """Run a server's serve() in a daemon thread (cf. gello sim_robot.py). Lets the
    MuJoCo follower keep the viewer on the main thread while ZMQ serves alongside."""

    def __init__(self, server: ZMQRobotServer):
        super().__init__(daemon=True)
        self._server = server

    def run(self) -> None:
        self._server.serve()

    def terminate(self) -> None:
        self._server.stop()
