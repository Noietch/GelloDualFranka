#!/usr/bin/env bash
# Bring up the official Franka ROS2 stack for the dual-arm rig: the C++ joint
# impedance controller + the Franka Hand gripper clients. Linux only.
#
# Requires franka_ros2 + the gello ros2 packages built/sourceable. See _ros_env.sh
# for how ROS2 workspaces are located (override with ROS_WS / ROS_DISTRO_SETUP).
#
# Edit robot_ip / namespace in the two config yamls before first run:
#   ros2/src/franka_fr3_arm_controllers/config/example_fr3_duo_config.yaml
#   ros2/src/franka_gripper_manager/config/example_fr3_duo_config_franka_hand.yaml
# namespaces (left/right) must match across controller, gripper, and the GELLO
# publisher (run_teleop_robot.sh).
set -e
[ "$(uname)" = "Linux" ] || { echo "real-robot stack is Linux only"; exit 1; }

DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "$DIR/_ros_env.sh"
ros_bootstrap || exit 1

ARM_CONFIG="${ARM_CONFIG:-example_fr3_config.yaml}"
GRIPPER_CONFIG="${GRIPPER_CONFIG:-example_fr3_config_franka_hand.yaml}"

echo "==> launching dual-arm impedance controllers ($ARM_CONFIG)"
ros2 launch franka_fr3_arm_controllers franka_fr3_arm_controllers.launch.py \
    robot_config_file:="$ARM_CONFIG" &
CTRL_PID=$!

# echo "==> launching Franka Hand gripper clients ($GRIPPER_CONFIG)"
# ros2 launch franka_gripper_manager franka_gripper_client.launch.py \
#     config_file:="$GRIPPER_CONFIG" &
# GRIP_PID=$!

trap 'echo; echo "stopping..."; kill $CTRL_PID $GRIP_PID 2>/dev/null || true' INT TERM
wait
