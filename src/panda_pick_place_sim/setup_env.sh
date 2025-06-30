#!/bin/bash
# setup_env.sh - ROS2 Jazzy + Local workspace 환경 설정 및 설치 가이드 스크립트

set -e

# ROS2환경 확인
echo "[1] Checking ROS 2 Jazzy installation..."
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "ROS 2 Jazzy not found. Install from: https://docs.ros.org/en/jazzy/"
  exit 1
fi
source /opt/ros/jazzy/setup.bash
echo "ROS 2 Jazzy sourced."

# workspace 확인
echo "[2] Locating workspace root..."
WORKSPACE_ROOT=$(ros2 pkg prefix panda_pick_place_sim 2>/dev/null | sed 's|/install.*||')
if [ -z "$WORKSPACE_ROOT" ]; then
  echo "Could not locate panda_pick_place_sim via ros2 pkg. Falling back to relative path."
  WORKSPACE_ROOT=$(realpath "$(dirname "$0")/../..")
fi
echo "Found workspace: $WORKSPACE_ROOT"

# colcon 확인
echo "[3] Checking for colcon..."
if ! command -v colcon &> /dev/null; then
  echo "   colcon not found. Please install with:"
  echo "    sudo apt install python3-colcon-common-extensions"
  exit 1
fi

# rosdep 확인
echo "[4] Checking for rosdep..."
if ! command -v rosdep &> /dev/null; then
  echo "   rosdep not found. Please install with:"
  echo "    sudo apt install python3-rosdep"
  exit 1
fi

# rosdep init 확인
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  echo "   rosdep not initialized. Run the following:"
  echo "    sudo rosdep init"
  echo "    rosdep update"
  exit 1
fi

# build 여부 확인
echo "[5] Checking if workspace is built..."
if [ ! -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
  echo "Workspace is not built."
  echo "   To build and install dependencies, run:"
  echo "    cd $WORKSPACE_ROOT"
  echo "    rosdep install --from-paths src --ignore-src -r -y"
  echo "    colcon build --packages-select panda_pick_place_sim"
  exit 1
fi

# build 환경 적용
source "$WORKSPACE_ROOT/install/setup.bash"
echo "Workspace sourced: $WORKSPACE_ROOT"

# rviz2 확인
echo "[6] Checking for RViz2..."
if ! command -v rviz2 &> /dev/null; then
  echo "   RViz2 is not installed. To install it, run:"
  echo "    sudo apt install ros-jazzy-rviz2"
else
  echo "RViz2 is installed."
fi

echo ""
echo "Environment is fully set up."
echo ""
echo "   Now you can launch the demo:"
echo "    ros2 launch panda_pick_place_sim demo.launch.py"
echo "    ros2 run panda_pick_place_sim main"
echo ""

# Keep terminal open
exec bash
