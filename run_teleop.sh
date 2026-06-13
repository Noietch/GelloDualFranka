#!/usr/bin/env bash
# 终端 2：启动 GELLO 遥操作客户端（连仿真节点 ZMQ :6001）。
# 读 configs/bimanual.yaml：left=none 只显示，right=GELLO 控制。Ctrl-C 停止。
set -euo pipefail
cd "$(dirname "$0")"

# 串口同一时刻只能被一个进程打开，先清掉残留的遥操作进程。
pkill -f run_bimanual.py 2>/dev/null || true

exec .venv/bin/python experiments/run_bimanual.py --config configs/bimanual.yaml "$@"
