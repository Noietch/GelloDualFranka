#!/usr/bin/env bash
# Shared ROS2 environment bootstrap, sourced by run_robot_stack.sh and
# run_teleop_robot.sh. Sourcing it (not executing) populates the current shell
# with the ROS2 distro + any overlay workspaces.
#
# Nothing here is guaranteed to exist on every machine, so each piece is detected
# and overridable. Override via environment variables:
#
#   ROS_DISTRO_SETUP   full path to /opt/ros/<distro>/setup.bash
#                      (default: auto-detect from $ROS_DISTRO, else first under /opt/ros)
#   ROS_WS             space-separated list of overlay workspaces to source AFTER
#                      the distro. Each entry may be:
#                        - a setup.bash / local_setup.bash file (sourced directly)
#                        - a workspace dir (we look for install/local_setup.bash,
#                          install/setup.bash, then <dir>/local_setup.bash)
#                      This is where franka_ros2 + the gello ros2 packages go.
#                      (default: the in-repo ros2/install workspace, if built)
#
# Examples:
#   ROS_WS="$HOME/franka_ros2_ws ../../ros2" bash run_robot_stack.sh
#   ROS_DISTRO_SETUP=/opt/ros/jazzy/setup.bash bash run_teleop_robot.sh

# --- locate the directory of THIS helper, so relative defaults are stable ---
# (works whether sourced from bash; falls back to $0 dir)
_RE_SELF="${BASH_SOURCE[0]:-$0}"
_RE_DIR="$(cd "$(dirname "$_RE_SELF")" && pwd)"

# --- 1. ROS2 distro -------------------------------------------------------
ros_find_distro_setup() {
  if [ -n "$ROS_DISTRO_SETUP" ] && [ -f "$ROS_DISTRO_SETUP" ]; then
    echo "$ROS_DISTRO_SETUP"; return 0
  fi
  if [ -n "$ROS_DISTRO" ] && [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    echo "/opt/ros/$ROS_DISTRO/setup.bash"; return 0
  fi
  # First match wins; if several distros are installed, set ROS_DISTRO_SETUP.
  for d in /opt/ros/*/setup.bash; do
    [ -f "$d" ] && { echo "$d"; return 0; }
  done
  return 1
}

# --- 2. resolve one overlay-workspace entry to a sourceable file ----------
ros_resolve_ws() {
  local entry="$1"
  if [ -f "$entry" ]; then echo "$entry"; return 0; fi
  if [ -d "$entry" ]; then
    for cand in \
      "$entry/install/local_setup.bash" \
      "$entry/install/setup.bash" \
      "$entry/local_setup.bash" \
      "$entry/setup.bash"; do
      [ -f "$cand" ] && { echo "$cand"; return 0; }
    done
  fi
  return 1
}

# --- 3. main entry: source distro + overlays into the current shell -------
# Usage: ros_bootstrap            (uses defaults / env overrides)
ros_bootstrap() {
  local distro_setup
  if ! distro_setup="$(ros_find_distro_setup)"; then
    echo "ERROR: no ROS2 distro found. Install ROS2 or set ROS_DISTRO_SETUP=" >&2
    echo "       e.g. ROS_DISTRO_SETUP=/opt/ros/humble/setup.bash" >&2
    return 1
  fi
  echo "==> ROS2 distro: $distro_setup"
  # shellcheck disable=SC1090
  source "$distro_setup"

  # Overlay workspaces: explicit ROS_WS, else default to the in-repo ros2 ws.
  local ws_list="$ROS_WS"
  if [ -z "$ws_list" ]; then
    ws_list="$_RE_DIR/../../ros2"
  fi

  local entry resolved
  for entry in $ws_list; do
    if resolved="$(ros_resolve_ws "$entry")"; then
      echo "==> overlay: $resolved"
      # shellcheck disable=SC1090
      source "$resolved"
    else
      echo "WARNING: workspace not found / not built: $entry" >&2
      echo "         (skipping; build it with 'colcon build' or fix ROS_WS)" >&2
    fi
  done
  return 0
}
