#!/usr/bin/env bash
# Minimal Norby workspace bootstrap.
#
# Prerequisites (install manually once):
#   - ROS 2 (Humble or Jazzy recommended)
#   - Webots + webots_ros2 for your ROS distro, e.g.:
#       sudo apt install ros-${ROS_DISTRO}-webots-ros2
#   - colcon: sudo apt install python3-colcon-common-extensions
#
# Usage:
#   source /opt/ros/<distro>/setup.bash
#   ./install.sh
#   source install/setup.bash

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

WEIGHTS_DIR="$ROOT/src/norby_webots_drivers/object_tracking/object_tracking/weights"
WEIGHTS_FILE="$WEIGHTS_DIR/coco-yolov4-tiny.weights"
WEIGHTS_URL="https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre/yolov4-tiny.weights"

log() { echo "==> $*"; }
warn() { echo "warning: $*" >&2; }
die() { echo "error: $*" >&2; exit 1; }

if [[ -z "${ROS_DISTRO:-}" ]]; then
  die "ROS 2 is not sourced. Run: source /opt/ros/<distro>/setup.bash"
fi

command -v python3 >/dev/null || die "python3 not found"
command -v colcon >/dev/null || die "colcon not found (sudo apt install python3-colcon-common-extensions)"

if [[ ! -d "$ROOT/src" ]]; then
  die "src/ directory not found. Run this script from the workspace root."
fi

if command -v rosdep >/dev/null; then
  log "Installing ROS dependencies with rosdep"
  rosdep update
  if ! rosdep install --from-paths src --ignore-src -r -y; then
    warn "rosdep reported missing packages."
    warn "If webots_ros2 or nav2 packages are missing, install them manually, e.g.:"
    warn "  sudo apt install ros-${ROS_DISTRO}-webots-ros2 ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-slam-toolbox"
  fi
else
  warn "rosdep not found; skipping ROS dependency resolution."
fi

log "Downloading YOLO weights (if missing)"
mkdir -p "$WEIGHTS_DIR"
if [[ -f "$WEIGHTS_FILE" ]]; then
  log "Weights already present: $WEIGHTS_FILE"
else
  if command -v curl >/dev/null; then
    curl -fL "$WEIGHTS_URL" -o "$WEIGHTS_FILE"
  elif command -v wget >/dev/null; then
    wget -O "$WEIGHTS_FILE" "$WEIGHTS_URL"
  else
    die "Need curl or wget to download YOLO weights."
  fi
  log "Saved weights to $WEIGHTS_FILE"
fi

log "Installing Python pip dependencies"
python3 -m pip install --upgrade pip
python3 -m pip install -r "$ROOT/requirements.txt"

log "Aligning Webots world files with sensor configuration"
python3 "$ROOT/scripts/update_worlds.py"

log "Building workspace with colcon"
colcon build --symlink-install

cat <<EOF

Norby install finished.

Next steps:
  source "$ROOT/install/setup.bash"
  ros2 launch norby_webots_drivers controllers.launch.py

Optional launches:
  ros2 launch norby_webots_drivers webots_only.launch.py
  ros2 launch norby_webots_drivers slam.launch.py
  ros2 launch norby_webots_drivers navigation.launch.py

EOF
