#!/usr/bin/env bash
# 步骤1: 仿真遥操。GELLO -> ZMQ -> MuJoCo 右臂动。无真机。
# 验证「读数 + 标定 + ZMQ 控制链」——这条链跟步骤3上真机时一模一样。
#
#   bash scripts/minimal_zmq/01_teleop_sim.sh            # 真 GELLO,实时窗口
#   LEADER=dummy bash scripts/minimal_zmq/01_teleop_sim.sh   # 无 GELLO,正弦兜底
#   ARM=left bash scripts/minimal_zmq/01_teleop_sim.sh   # 改控制左臂
#
# Mac 可视化两种方式:
#   - 实时窗口(默认):弹 MuJoCo 窗口交互看(Mac 用 mjpython)。
#   - 离屏录制:    RECORD=out.mp4 [SECS=10] bash 01_teleop_sim.sh
#                  无需窗口/mjpython,渲染成 MP4 回放(适合无显示器/录演示)。
#
# 本脚本同时起两个进程: follower_sim(server,前台) + teleop(client,后台)。
# 关窗/录制结束/Ctrl-C 一并退出。标定用 sim(modulo-wrap),由本脚本固定传入。
set -e
DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "$DIR/_find_venv.sh"

ARM="${ARM:-right}"
LEADER="${LEADER:-gello}"
RECORD="${RECORD:-}"
SECS_REC="${SECS:-10}"
PORT="${PORT:-}"   # 覆盖 config 里的 ZMQ 端口(默认按 arm 取);端口被占时可用

PORT_ARGS=()
[ -n "$PORT" ] && PORT_ARGS=(--port "$PORT")

PY="$(find_python "$DIR")" || { echo "缺 python；先 bash $DIR/00_setup.sh"; exit 1; }

if [ -n "$RECORD" ]; then
  # 离屏录制: follower_sim 无窗口渲染成 MP4,用普通 python 即可。
  "$PY" "$DIR/follower_sim.py" --arm "$ARM" "${PORT_ARGS[@]}" --record "$RECORD" --seconds "$SECS_REC" &
  SIM_PID=$!
else
  # 实时窗口: Mac 必须 mjpython。
  if [ "$(uname)" = "Darwin" ]; then
    GUI_PY="$(find_python "$DIR" mjpython)" || { echo "缺 mjpython；先 bash $DIR/00_setup.sh"; exit 1; }
  else
    GUI_PY="$PY"
  fi
  "$GUI_PY" "$DIR/follower_sim.py" --arm "$ARM" "${PORT_ARGS[@]}" &
  SIM_PID=$!
fi

trap 'kill $SIM_PID $TELE_PID 2>/dev/null || true' INT TERM EXIT
sleep 2

"$PY" "$DIR/teleop.py" --arm "$ARM" --calib sim --leader "$LEADER" "${PORT_ARGS[@]}" &
TELE_PID=$!

wait $SIM_PID
