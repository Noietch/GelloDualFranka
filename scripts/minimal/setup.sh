#!/usr/bin/env bash
# Minimal GELLO environment setup. Creates a .venv next to this script and
# installs the pure-Python deps + the in-repo DynamixelSDK. Uses uv if present,
# otherwise falls back to the stdlib venv + pip (uv is NOT required).
#
#   bash scripts/minimal/setup.sh
#   PYTHON=python3.11 bash scripts/minimal/setup.sh   # pick the interpreter
#
# rclpy is NOT installed here — the real-robot modes use the system ROS2.
set -e
cd "$(dirname "$0")"

DXL_DIR="../../third_party/DynamixelSDK/python"
if [ ! -f "$DXL_DIR/setup.py" ]; then
  echo "ERROR: DynamixelSDK submodule not found at third_party/DynamixelSDK."
  echo "Run this first, then re-run setup:"
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
  ver="$("$PYBIN" -c 'import sys;print("%d.%d"%sys.version_info[:2])')"
  case "$ver" in
    3.1[0-9]) : ;;  # 3.10+
    *) echo "WARNING: python $ver detected; 3.10+ recommended (mujoco/dm_control)." ;;
  esac
  "$PYBIN" -m venv .venv
  # shellcheck disable=SC1091
  source .venv/bin/activate
  pip install --upgrade pip
  pip install -r requirements.txt
  pip install -e "$DXL_DIR"
fi

# macOS: mujoco's mjpython needs libpython on the rpath. venvs built from a
# Python whose libpython isn't on the loader path fail to dlopen — symlink it in.
if [ "$(uname)" = "Darwin" ]; then
  prefix="$(python -c 'import sys;print(sys.base_prefix)')"
  pyver="$(python -c 'import sys;print("%d.%d"%sys.version_info[:2])')"
  lib="$(ls "$prefix"/lib/libpython"$pyver"*.dylib 2>/dev/null | head -1 || true)"
  if [ -n "$lib" ]; then
    ln -sf "$lib" ".venv/$(basename "$lib")"
    echo "==> linked $(basename "$lib") into .venv (mjpython dlopen fix)"
  else
    echo "WARNING: libpython$pyver dylib not found under $prefix/lib;"
    echo "         if mjpython fails to start, create the symlink manually."
  fi
fi

echo ""
echo "done."
echo "  activate: source scripts/minimal/.venv/bin/activate"
echo "  sim:      mjpython scripts/minimal/main.py teleop-sim   # (linux: python ...)"
