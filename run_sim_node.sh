#!/usr/bin/env bash
# 终端 1：启动 MuJoCo 双臂 Panda 仿真节点（ZMQ 服务端 :6001）。
# 必须用 mjpython，前台运行，会弹出 MuJoCo 窗口。Ctrl-C 停止。
set -euo pipefail
cd "$(dirname "$0")"

# ZMQ :6001 同一时刻只能被一个节点占用，先清掉残留的仿真节点。
pkill -9 -f launch_nodes.py 2>/dev/null || true
sleep 1

exec .venv/bin/mjpython experiments/launch_nodes.py --robot sim_bimanual_panda
