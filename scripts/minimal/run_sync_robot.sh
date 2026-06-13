#!/usr/bin/env bash
# 模式1: 真机 ROS2 关节状态 -> MuJoCo（单向镜像，可视化/调试用）。
# 需要系统 ROS2 + franka_ros2，且真机控制器已在发布 <ns>/franka/joint_states。
# 实际几乎只在 Linux 上用（Mac 没有 ROS2）。
#
#   bash scripts/minimal/run_sync_robot.sh
#
# ROS2 工作区的查找/覆盖见 _ros_env.sh（ROS_WS / ROS_DISTRO_SETUP）。
set -e
DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "$DIR/_find_venv.sh"

# 先 source ROS2（拿到 rclpy/sensor_msgs），再选解释器。
# shellcheck disable=SC1091
source "$DIR/_ros_env.sh"
ros_bootstrap || exit 1

# MuJoCo 窗口在 Mac 上必须 mjpython；Linux 用普通 python。
# venv 找不到则回退系统 python3（容器里依赖已装进系统）。
if [ "$(uname)" = "Darwin" ]; then
  PY="$(find_python "$DIR" mjpython)" || { echo "未找到 mjpython（venv 或系统均无）；请先 bash $DIR/setup.sh"; exit 1; }
else
  PY="$(find_python "$DIR")" || { echo "未找到 python（venv 或系统均无）；请先 bash $DIR/setup.sh"; exit 1; }
fi

exec "$PY" "$DIR/main.py" sync-robot
