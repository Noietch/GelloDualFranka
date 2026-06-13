#!/usr/bin/env bash
# 共享 helper: 定位 Python 解释器。被各步骤脚本 source。
# 优先用本目录 .venv,找不到回退系统 python3。
#   source _find_venv.sh
#   PY="$(find_python "$DIR")"            # 普通 python
#   PY="$(find_python "$DIR" mjpython)"   # macOS MuJoCo 启动器
find_venv() {
  local script_dir="$1"
  local candidates=("$MINIMAL_ZMQ_VENV" "$script_dir/.venv")
  local c
  for c in "${candidates[@]}"; do
    [ -n "$c" ] && [ -x "$c/bin/python" ] && { echo "$c"; return 0; }
  done
  return 1
}

find_python() {
  local script_dir="$1"
  local want="${2:-python}"
  local venv
  if venv="$(find_venv "$script_dir")" && [ -x "$venv/bin/$want" ]; then
    echo "$venv/bin/$want"
    return 0
  fi
  local sys
  for sys in "$want" "${want}3"; do
    if command -v "$sys" >/dev/null 2>&1; then
      command -v "$sys"
      return 0
    fi
  done
  return 1
}
