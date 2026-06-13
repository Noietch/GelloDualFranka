#!/usr/bin/env bash
# 步骤3: 真机遥操。GELLO -> ZMQ -> franky 真机。
# 前两步都通过后再跑这步。teleop 客户端与步骤1完全相同,只是另一端从 MuJoCo
# 换成 franky 真机;标定改用 real(增量+限位),由本脚本固定传入。
#
#   bash scripts/minimal_zmq/03_teleop_robot.sh
#   RELDYN=0.05 bash scripts/minimal_zmq/03_teleop_robot.sh   # franky 速度更慢更稳
#
# 真机 follower 仅限 Linux + 实时内核,且需 pip install franky-control。
# 安全: teleop 启动会先比较 GELLO 与真机当前位姿,差太大直接中止(不会突然弹跳)。
set -e
[ "$(uname)" = "Linux" ] || { echo "步骤3(franky 真机)仅限 Linux"; exit 1; }
DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "$DIR/_find_venv.sh"

ARM="${ARM:-right}"
RELDYN="${RELDYN:-0.08}"
PY="$(find_python "$DIR")" || { echo "缺 python；先 bash $DIR/00_setup.sh"; exit 1; }

# 先起 franky follower(控制模式,server),再起 teleop(GELLO client)。
"$PY" "$DIR/follower.py" --arm "$ARM" --relative-dynamics "$RELDYN" &
FOLLOWER_PID=$!
trap 'kill $FOLLOWER_PID $TELE_PID 2>/dev/null || true' INT TERM EXIT
sleep 3

"$PY" "$DIR/teleop.py" --arm "$ARM" --calib real --leader gello &
TELE_PID=$!
wait $TELE_PID
