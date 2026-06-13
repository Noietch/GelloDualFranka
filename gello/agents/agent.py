from typing import Any, Dict, Protocol

import numpy as np


class Agent(Protocol):
    def act(self, obs: Dict[str, Any]) -> np.ndarray:
        """Returns an action given an observation.

        Args:
            obs: observation from the environment.

        Returns:
            action: action to take on the environment.
        """
        raise NotImplementedError


class DummyAgent(Agent):
    def __init__(self, num_dofs: int):
        self.num_dofs = num_dofs

    def act(self, obs: Dict[str, Any]) -> np.ndarray:
        return np.zeros(self.num_dofs)


class PassiveAgent(Agent):
    """Holds an uncontrolled ('none') arm at a fixed pose so it is shown but
    never moves. If hold_pose is given, the arm is driven to and held there;
    otherwise it holds whatever pose it starts at."""

    def __init__(self, num_dofs: int, hold_pose=None):
        self.num_dofs = num_dofs
        self.hold_pose = None if hold_pose is None else np.asarray(hold_pose, dtype=float)

    def act(self, obs: Dict[str, Any]) -> np.ndarray:
        if self.hold_pose is not None:
            return self.hold_pose[: self.num_dofs]
        return np.asarray(obs["joint_positions"])[: self.num_dofs]


class BimanualAgent(Agent):
    def __init__(self, agent_left: Agent, agent_right: Agent):
        self.agent_left = agent_left
        self.agent_right = agent_right

    def act(self, obs: Dict[str, Any]) -> np.ndarray:
        left_obs = {}
        right_obs = {}
        for key, val in obs.items():
            val = np.asarray(val)
            if val.ndim == 0:
                continue  # scalar obs (e.g. gripper_position) — not per-arm
            L = val.shape[0]
            half_dim = L // 2
            # Only split per-arm even-length vectors (e.g. joint_positions).
            # Skip odd/aggregate keys like ee_pos_quat that aren't per-arm.
            if L != half_dim * 2:
                continue
            left_obs[key] = val[:half_dim]
            right_obs[key] = val[half_dim:]
        return np.concatenate(
            [self.agent_left.act(left_obs), self.agent_right.act(right_obs)]
        )
