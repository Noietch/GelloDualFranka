#!/usr/bin/env bash
# 共享 helper: 定位 Python 解释器。被四个 run_*.sh source。
# 优先用 venv（裸机/Mac），找不到则回退到系统 python3（容器里 rclpy/依赖
# 已装进系统 Python，无需 venv）。
#
# 用法:
#   source _find_venv.sh
#   PY="$(find_python "$SCRIPT_DIR")"          # 普通 python
#   PY="$(find_python "$SCRIPT_DIR" mjpython)" # macOS MuJoCo 启动器

# 定位 venv 目录。查找顺序:
#   1. $GELLO_VENV            （环境变量显式指定，最高优先级）
#   2. scripts/minimal/.venv  （setup.sh 默认创建位置）
#   3. <仓库根>/.venv         （兼容已有的仓库根 venv）
# 找到第一个含 bin/python 的就返回其路径；都没有则返回非零。
find_venv() {
  local script_dir="$1"
  local repo_root
  repo_root="$(cd "$script_dir/../.." && pwd)"
  local candidates=(
    "$GELLO_VENV"
    "$script_dir/.venv"
    "$repo_root/.venv"
  )
  local c
  for c in "${candidates[@]}"; do
    [ -n "$c" ] && [ -x "$c/bin/python" ] && { echo "$c"; return 0; }
  done
  return 1
}

# 解析出要用的 Python 解释器路径。
#   $1 = script_dir
#   $2 = 解释器名，默认 "python"；传 "mjpython" 取 macOS MuJoCo 启动器。
# 先看 venv 里有没有对应可执行；没有则在系统 PATH 上回退（python -> python3）。
# 打印解释器路径并返回 0；都找不到返回非零。
find_python() {
  local script_dir="$1"
  local want="${2:-python}"
  local venv
  if venv="$(find_venv "$script_dir")" && [ -x "$venv/bin/$want" ]; then
    echo "$venv/bin/$want"
    return 0
  fi
  # 系统回退: 容器/已全局安装依赖的环境走这里。
  local sys
  for sys in "$want" "${want}3"; do
    if command -v "$sys" >/dev/null 2>&1; then
      command -v "$sys"
      return 0
    fi
  done
  return 1
}
