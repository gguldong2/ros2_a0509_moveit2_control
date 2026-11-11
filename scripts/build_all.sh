#!/bin/bash
# Build every program in this repo (CLI tools + Qt GUI). Usage: scripts/build_all.sh [DRCF_VERSION]
set -e
REPO="$(cd "$(dirname "$0")/.." && pwd)"
VER="${1:-2}"
cd "$REPO"
for f in tests/connect_test.cpp tests/move_test.cpp tests/joint_test.cpp \
         a0509_menu/a0509_menu.cpp gripper/gripper_test.cpp gripper/gripper_scan.cpp; do
  ./build.sh "$f" "$VER"
done
cmake -S a0509_gui -B a0509_gui/build -DDRCF_VERSION="$VER" ${DRFL_ROOT:+-DDRFL_ROOT="$DRFL_ROOT"} >/dev/null
cmake --build a0509_gui/build -j
a0509_gui/build/pose_math_test   # offline check of the MoveL pose math (RPY <-> ZYZ)
echo "GUI: a0509_gui/build/a0509_gui"
