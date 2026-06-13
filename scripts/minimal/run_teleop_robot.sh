#!/usr/bin/env bash
# 模式3: GELLO -> 真机。读两个 GELLO，用「增量积分 + FR3 限位」标定，按臂发布
# <ns>/gello/joint_states (+ 夹爪) 给官方 C++ 阻抗控制器。Linux only。
#
# 启动顺序: 1) bash run_robot_stack.sh   2) bash run_teleop_robot.sh
#
# source 顺序很重要: 先 ROS2（rclpy/sensor_msgs/std_msgs），再 venv
# （dynamixel_sdk/numpy），两层叠加在 PYTHONPATH 上。
# ROS2 工作区查找/覆盖见 _ros_env.sh（ROS_WS / ROS_DISTRO_SETUP）。
set -e
[ "$(uname)" = "Linux" ] || { echo "teleop-robot 仅限 Linux"; exit 1; }

DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "$DIR/_find_venv.sh"

# shellcheck disable=SC1091
source "$DIR/_ros_env.sh"
ros_bootstrap || exit 1

# 有 venv 就 activate（裸机：把 dynamixel_sdk/numpy 叠到系统 ROS2 之上）；
# 没有则回退系统 python3（容器里这些依赖已装进系统 Python）。
if VENV="$(find_venv "$DIR")"; then
  # shellcheck disable=SC1090
  source "$VENV/bin/activate"
fi
PY="$(find_python "$DIR")" || { echo "未找到 python（venv 或系统均无）；请先 bash $DIR/setup.sh"; exit 1; }

exec "$PY" "$DIR/main.py" teleop-robot
