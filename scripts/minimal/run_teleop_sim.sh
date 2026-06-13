#!/usr/bin/env bash
# 模式2: GELLO -> MuJoCo 双臂仿真遥操作。
# 不需要 ROS2，不需要真机。Mac 和 Linux 都能跑。
#
#   bash scripts/minimal/run_teleop_sim.sh
#
# 前置: 裸机先跑过 setup.sh 建好 .venv；容器里依赖已装进系统 Python 则无需 venv。
# 解释器: 默认探测 venv（GELLO_VENV / scripts/minimal/.venv / 仓库根 .venv），
# 找不到则回退系统 python3。
set -e
DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "$DIR/_find_venv.sh"

# MuJoCo 窗口在 Mac 上必须用 mjpython 启动；Linux 用普通 python。
if [ "$(uname)" = "Darwin" ]; then
  PY="$(find_python "$DIR" mjpython)" || { echo "未找到 mjpython（venv 或系统均无）；请先 bash $DIR/setup.sh"; exit 1; }
else
  PY="$(find_python "$DIR")" || { echo "未找到 python（venv 或系统均无）；请先 bash $DIR/setup.sh"; exit 1; }
fi

exec "$PY" "$DIR/main.py" teleop-sim
