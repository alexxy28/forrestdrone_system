#!/usr/bin/env bash

set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$ROOT_DIR/ros2_simulation"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"

WORLD="terrain_demo"
HUSKY="true"
PARROT="true"
RVIZ="true"
NAV2="true"
SLAM="false"
LOCAL_ONLY="1"
BUILD="true"
CLEAN_BUILD="false"
ROS_DOMAIN=""

usage() {
  cat <<'EOF'
Usage: ./run_simulation.sh [options]

Build, source, and start the Forrest Drone Ignition simulation.

Options:
  -w, --world WORLD     World: simple_trees, large_demo, or terrain_demo
      --husky-only      Launch only the Husky
      --parrot-only     Launch only the Parrot
      --no-rviz         Do not launch RViz
      --nav2            Enable Nav2
      --no-nav2         Disable Nav2
      --slam            Enable SLAM Toolbox
      --network         Allow ROS 2 discovery over the network
      --local           Restrict ROS 2 traffic to this computer (default)
      --domain-id ID    Set ROS_DOMAIN_ID
      --no-build        Use the existing workspace build
      --clean-build     Remove build artifacts before rebuilding
  -h, --help            Show this help

Defaults: terrain_demo, both robots, RViz, Nav2, local-only ROS, rebuild enabled.
EOF
}

while (($# > 0)); do
  case "$1" in
    -w|--world)
      [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 2; }
      WORLD="$2"
      shift 2
      ;;
    --husky-only)
      HUSKY="true"
      PARROT="false"
      shift
      ;;
    --parrot-only)
      HUSKY="false"
      PARROT="true"
      shift
      ;;
    --no-rviz)
      RVIZ="false"
      shift
      ;;
    --nav2)
      NAV2="true"
      shift
      ;;
    --no-nav2)
      NAV2="false"
      shift
      ;;
    --slam)
      SLAM="true"
      shift
      ;;
    --network)
      LOCAL_ONLY="0"
      shift
      ;;
    --local)
      LOCAL_ONLY="1"
      shift
      ;;
    --domain-id)
      [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 2; }
      ROS_DOMAIN="$2"
      shift 2
      ;;
    --no-build)
      BUILD="false"
      shift
      ;;
    --clean-build)
      CLEAN_BUILD="true"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "$WORLD" in
  simple_trees|large_demo|terrain_demo) ;;
  *)
    echo "Invalid world '$WORLD'. Use simple_trees, large_demo, or terrain_demo." >&2
    exit 2
    ;;
esac

if [[ -n "$ROS_DOMAIN" && ! "$ROS_DOMAIN" =~ ^[0-9]+$ ]]; then
  echo "ROS domain ID must be a non-negative integer." >&2
  exit 2
fi

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS setup file not found: $ROS_SETUP" >&2
  echo "Set ROS_SETUP to the setup.bash for your ROS distribution." >&2
  exit 1
fi

if [[ ! -d "$WORKSPACE/src" ]]; then
  echo "ROS workspace not found: $WORKSPACE" >&2
  exit 1
fi

set +u
source "$ROS_SETUP"
set -u

if [[ "$CLEAN_BUILD" == "true" ]]; then
  rm -rf "$WORKSPACE/build" "$WORKSPACE/install" "$WORKSPACE/log"
fi

if [[ "$BUILD" == "true" ]]; then
  command -v colcon >/dev/null || {
    echo "colcon is not installed or is not on PATH." >&2
    exit 1
  }
  echo "Building ROS workspace..."
  (cd "$WORKSPACE" && colcon build --symlink-install)
fi

if [[ ! -f "$WORKSPACE/install/setup.bash" ]]; then
  echo "Workspace setup file is missing. Run without --no-build first." >&2
  exit 1
fi

set +u
source "$WORKSPACE/install/setup.bash"
set -u
export ROS_LOCALHOST_ONLY="$LOCAL_ONLY"
if [[ -n "$ROS_DOMAIN" ]]; then
  export ROS_DOMAIN_ID="$ROS_DOMAIN"
fi

echo "Starting simulation:"
echo "  world=$WORLD husky=$HUSKY parrot=$PARROT rviz=$RVIZ nav2=$NAV2 slam=$SLAM"
echo "  ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"

exec ros2 launch 41068_ignition_bringup 41068_ignition.launch.py \
  world:="$WORLD" \
  husky:="$HUSKY" \
  parrot:="$PARROT" \
  rviz:="$RVIZ" \
  nav2:="$NAV2" \
  slam:="$SLAM"
