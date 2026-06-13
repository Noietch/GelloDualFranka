#!/usr/bin/env bash
# 步骤2: 真机状态可视化(只读,不发指令 — 最安全)。
# franky 只读 FR3 关节状态 -> ZMQ -> MuJoCo 镜像显示。确认 franky 连得上、
# 真机关节角和仿真对得上,再进步骤3。本步骤绝不会让机器人动。
#
# 单机(robot PC 带显示器):
#   bash scripts/minimal_zmq/02_visualize_robot.sh
# 分机(robot PC 跑 franky,Mac 看):
#   robot PC:  python follower.py --arm right --read-only --host 0.0.0.0
#   Mac:       HOST=<robot_pc_ip> bash scripts/minimal_zmq/02_visualize_robot.sh
#
# 真机 follower 仅限 Linux + 实时内核,且需 pip install franky-control。
set -e
DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "$DIR/_find_venv.sh"

ARM="${ARM:-right}"
HOST="${HOST:-127.0.0.1}"

if [ "$(uname)" = "Darwin" ]; then
  GUI_PY="$(find_python "$DIR" mjpython)" || { echo "缺 mjpython；先 bash $DIR/00_setup.sh"; exit 1; }
else
  GUI_PY="$(find_python "$DIR")" || { echo "缺 python；先 bash $DIR/00_setup.sh"; exit 1; }
fi
PY="$(find_python "$DIR")" || { echo "缺 python；先 bash $DIR/00_setup.sh"; exit 1; }

# 仅当连本机时,顺带在后台起 franky follower(只读)。连远程则跳过。
FOLLOWER_PID=""
if [ "$HOST" = "127.0.0.1" ] || [ "$HOST" = "localhost" ]; then
  [ "$(uname)" = "Linux" ] || { echo "真机 follower 仅限 Linux；Mac 请用分机模式(见脚本注释)"; exit 1; }
  "$PY" "$DIR/follower.py" --arm "$ARM" --read-only &
  FOLLOWER_PID=$!
  sleep 2
fi
trap 'kill $FOLLOWER_PID $MIRROR_PID 2>/dev/null || true' INT TERM EXIT

"$GUI_PY" "$DIR/mirror.py" --arm "$ARM" --host "$HOST" &
MIRROR_PID=$!
wait $MIRROR_PID
