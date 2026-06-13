#!/usr/bin/env bash
# 步骤0: 安装环境。在本目录建独立 .venv,装 numpy/pyzmq/mujoco/dm_control +
# 仓库内置的 DynamixelSDK。优先用 uv,没有则回退 venv+pip。
#
#   bash scripts/minimal_zmq/00_setup.sh
#   PYTHON=python3.11 bash scripts/minimal_zmq/00_setup.sh   # 指定解释器
#
# franky 不在这里装(仅 Linux+实时内核,步骤2/3 真机用):到真机 PC 上单独
#   pip install franky-control
set -e
cd "$(dirname "$0")"

DXL_DIR="../../third_party/DynamixelSDK/python"
if [ ! -f "$DXL_DIR/setup.py" ]; then
  echo "ERROR: 未找到 DynamixelSDK 子模块（third_party/DynamixelSDK）。"
  echo "先执行,再重跑本脚本:"
  echo "    git submodule update --init third_party/DynamixelSDK"
  exit 1
fi

PYBIN="${PYTHON:-python3}"

if command -v uv >/dev/null 2>&1; then
  echo "==> using uv"
  uv venv --python 3.11 .venv
  # shellcheck disable=SC1091
  source .venv/bin/activate
  uv pip install -r requirements.txt
  uv pip install -e "$DXL_DIR"
else
  echo "==> uv not found, falling back to venv + pip ($PYBIN)"
  "$PYBIN" -m venv .venv
  # shellcheck disable=SC1091
  source .venv/bin/activate
  pip install --upgrade pip
  pip install -r requirements.txt
  pip install -e "$DXL_DIR"
fi

# macOS: mujoco 的 mjpython 需要 libpython 在 rpath 上,某些 venv 找不到会
# dlopen 失败 — 把 libpython 软链进 .venv。
if [ "$(uname)" = "Darwin" ]; then
  prefix="$(python -c 'import sys;print(sys.base_prefix)')"
  pyver="$(python -c 'import sys;print("%d.%d"%sys.version_info[:2])')"
  lib="$(ls "$prefix"/lib/libpython"$pyver"*.dylib 2>/dev/null | head -1 || true)"
  if [ -n "$lib" ]; then
    ln -sf "$lib" ".venv/$(basename "$lib")"
    echo "==> linked $(basename "$lib") into .venv (mjpython dlopen fix)"
  else
    echo "WARNING: 未找到 libpython$pyver dylib；若 mjpython 启动失败请手动建软链。"
  fi
fi

echo ""
echo "done。下一步:"
echo "  步骤1 仿真遥操:  bash scripts/minimal_zmq/01_teleop_sim.sh"
