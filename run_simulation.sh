#!/usr/bin/env bash

set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$ROOT_DIR/ros2_simulation"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"

WORLD="terrain_demo"
HUSKY="true"
PARROT="true"
RVIZ="true"
GUI="true"
NAV2="true"
SLAM="false"
LOCAL_ONLY="1"
BUILD="true"
CLEAN_BUILD="false"
ROS_DOMAIN=""
TRACE="true"
POSE=""
PRELOAD_MAP="false"
DASHBOARD="true"
LOGGER="true"
START_PAUSED="true"
PARROT2="false"
DRONE_SEARCH="race"
SHOW_MENU="false"
SETUP="false"
[[ $# -gt 0 ]] || SHOW_MENU="true"

usage() {
  cat <<'EOF'
Usage: ./run_simulation.sh [options]

Build, source, and start the full TRACE mission simulation.
With no arguments, open the interactive menu.

Options:
      --menu            Open the interactive menu
      --setup           Update from GitHub, install dependencies, and build
  -w, --world WORLD     World: simple_trees, large_demo, or terrain_demo
      --husky-only      Launch only the Husky
      --parrot-only     Launch only the Parrot
      --no-rviz         Do not launch RViz
      --no-gui          Run Gazebo headless (server only, less CPU)
      --nav2            Enable Nav2
      --no-nav2         Disable Nav2
      --slam            Enable SLAM Toolbox
      --network         Allow ROS 2 discovery over the network
      --local           Restrict ROS 2 traffic to this computer (default)
      --domain-id ID    Set ROS_DOMAIN_ID
      --no-build        Use the existing workspace build
      --clean-build     Remove build artifacts before rebuilding
      --trace           Full TRACE mission (trace_demo.launch.py): sim, SLAM,
                        control nodes, rqt dashboard, results logger (default)
      --sim-only        Simulator and robots without TRACE mission nodes/dashboard
      --no-dashboard    Disable the rqt dashboard (TRACE)
      --no-logger       Disable mission scoring/results logging (TRACE)
      --auto-takeoff    Start the Parrot without pressing Start Parrot (TRACE)
      --second-drone    Add a second Parrot (/parrot2). With TRACE both drones
                        race over the whole area on different paths
      --split-search    With --second-drone: each drone searches half instead
      --pose N          Casualty spawn 0-5 (default: random)
      --preload-map[=true|false]
                        With --trace: start from the saved terrain map for the
                        world (pre_scanned_height_maps/WORLD)
  -h, --help            Show this help

Defaults: full TRACE mission, terrain_demo, both robots, RViz, Nav2,
          local-only ROS, rebuild enabled.
EOF
}

toggle() {
  if [[ "${!1}" == "true" ]]; then
    printf -v "$1" '%s' false
  else
    printf -v "$1" '%s' true
  fi
}

simulation_menu() {
  local choice value
  while true; do
    cat <<EOF

Simulation settings (true = selected)
  1. Full TRACE mission:                    $TRACE
  2. Gazebo window:                         $GUI
  3. RViz:                                  $RVIZ
  4. Nav2 (also enables SLAM):               $NAV2
  5. SLAM without Nav2:                      $SLAM
  6. Load saved terrain map (TRACE):         $PRELOAD_MAP
  7. World:                                 $WORLD
  8. Casualty pose:                         ${POSE:-random}
  9. Robots (sim-only): Husky=$HUSKY Parrot=$PARROT
 10. Build before launching:                 $BUILD
 11. Local-only ROS:                         $LOCAL_ONLY
 12. Operator dashboard (TRACE):             $DASHBOARD
 13. Results logger (TRACE):                 $LOGGER
 14. Wait for Start Parrot (TRACE):           $START_PAUSED
 15. Second drone:                           $PARROT2
 16. Two-drone search (race/split):          $DRONE_SEARCH
  s. Start simulation
  b. Back

TRACE always runs both robots, Nav2 and SLAM.
EOF
    read -r -p 'Select an option: ' choice || return 1
    case "$choice" in
      1) toggle TRACE; [[ "$TRACE" == true ]] || PRELOAD_MAP=false ;;
      2) toggle GUI ;;
      3) toggle RVIZ ;;
      4) toggle NAV2 ;;
      5) toggle SLAM ;;
      6)
        if [[ "$TRACE" == true ]]; then toggle PRELOAD_MAP
        else echo 'Saved terrain maps require the full TRACE mission.'; fi ;;
      7)
        read -r -p 'World (1 terrain_demo, 2 simple_trees, 3 large_demo): ' value || return 1
        case "$value" in
          1) WORLD=terrain_demo ;; 2) WORLD=simple_trees ;; 3) WORLD=large_demo ;;
          *) echo 'Invalid world selection.' ;;
        esac ;;
      8)
        read -r -p 'Casualty pose (0-5, or r for random): ' value || return 1
        case "$value" in
          [0-5]) POSE="$value" ;; r|R) POSE='' ;; *) echo 'Invalid pose.' ;;
        esac ;;
      9)
        read -r -p 'Robots (1 both, 2 Husky only, 3 Parrot only): ' value || return 1
        case "$value" in
          1) HUSKY=true; PARROT=true ;;
          2) HUSKY=true; PARROT=false ;;
          3) HUSKY=false; PARROT=true ;;
          *) echo 'Invalid robot selection.' ;;
        esac ;;
      10) toggle BUILD ;;
      11) if [[ "$LOCAL_ONLY" == 1 ]]; then LOCAL_ONLY=0; else LOCAL_ONLY=1; fi ;;
      12) toggle DASHBOARD ;;
      13) toggle LOGGER ;;
      14) toggle START_PAUSED ;;
      15) toggle PARROT2 ;;
      16) if [[ "$DRONE_SEARCH" == race ]]; then DRONE_SEARCH=split; else DRONE_SEARCH=race; fi ;;
      s|S) return 0 ;;
      b|B) return 1 ;;
      *) echo 'Invalid selection.' ;;
    esac
  done
}

main_menu() {
  local choice
  while true; do
    cat <<'EOF'

TRACE simulation
  1. Configure add-ons and run simulation
  2. Update from GitHub and install/setup dependencies
  0. Exit
EOF
    read -r -p 'Select an option: ' choice || exit 0
    case "$choice" in
      1) if simulation_menu; then return; fi ;;
      2) SETUP=true; return ;;
      0) exit 0 ;;
      *) echo 'Invalid selection.' ;;
    esac
  done
}

while (($# > 0)); do
  case "$1" in
    --menu) SHOW_MENU=true; shift ;;
    --setup) SETUP=true; shift ;;
    --no-dashboard) DASHBOARD=false; shift ;;
    --no-logger) LOGGER=false; shift ;;
    --auto-takeoff) START_PAUSED=false; shift ;;
    --second-drone) PARROT2=true; shift ;;
    --split-search) DRONE_SEARCH=split; shift ;;
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
    --no-gui)
      GUI="false"
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
    --trace)
      TRACE="true"
      shift
      ;;
    --sim-only)
      TRACE="false"
      shift
      ;;
    --pose)
      [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 2; }
      POSE="$2"
      shift 2
      ;;
    --preload-map|--preload-map=true)
      PRELOAD_MAP="true"
      shift
      ;;
    --preload-map=false)
      PRELOAD_MAP="false"
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

if [[ "$SHOW_MENU" == true && "$SETUP" == false ]]; then
  main_menu
fi
if [[ "$SETUP" == true ]]; then
  exec bash "$ROOT_DIR/scripts/setup_environment.sh"
fi

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

if [[ -n "$POSE" && ! "$POSE" =~ ^[0-5]$ ]]; then
  echo "Pose must be 0-5." >&2
  exit 2
fi

if [[ "$PRELOAD_MAP" == "true" && "$TRACE" != "true" ]]; then
  echo "--preload-map needs --trace (the map is loaded by the control nodes)." >&2
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

POSE_ARGS=()
[[ -n "$POSE" ]] && POSE_ARGS=(human_pose:="$POSE")
MAP_ARGS=()
[[ "$PRELOAD_MAP" == "true" ]] && MAP_ARGS=(height_map:="$WORLD")

if [[ "$TRACE" == "true" ]]; then
  echo "Starting TRACE mission: world=$WORLD rviz=$RVIZ gui=$GUI human_pose=${POSE:-random} preload_map=$PRELOAD_MAP"
  [[ "$PARROT2" == "true" ]] && echo "  second drone: on, search=$DRONE_SEARCH"
  echo "  ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
  # Robots, SLAM and Nav2 are fixed on inside trace_demo, so those flags are ignored here.
  exec ros2 launch 41068_ignition_bringup trace_demo.launch.py \
    world:="$WORLD" \
    rviz:="$RVIZ" \
    gui:="$GUI" \
    rqt:="$DASHBOARD" \
    log_results:="$LOGGER" \
    parrot_start_paused:="$START_PAUSED" \
    parrot2:="$PARROT2" \
    drone_search:="$DRONE_SEARCH" \
    "${POSE_ARGS[@]}" \
    "${MAP_ARGS[@]}"
fi

echo "Starting simulation:"
echo "  world=$WORLD husky=$HUSKY parrot=$PARROT parrot2=$PARROT2 rviz=$RVIZ gui=$GUI nav2=$NAV2 slam=$SLAM"
echo "  ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"

exec ros2 launch 41068_ignition_bringup 41068_ignition.launch.py \
  world:="$WORLD" \
  husky:="$HUSKY" \
  parrot:="$PARROT" \
  parrot2:="$PARROT2" \
  rviz:="$RVIZ" \
    gui:="$GUI" \
  nav2:="$NAV2" \
  slam:="$SLAM" \
  "${POSE_ARGS[@]}"
