#!/bin/bash
# Clone the official Doosan Robotics DRFL SDK (BSD-3-Clause) into third_party/API-DRFL.
set -e
REPO="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$REPO/third_party"
if [ -d "$REPO/third_party/API-DRFL/.git" ]; then
  git -C "$REPO/third_party/API-DRFL" pull --ff-only
else
  git clone --depth 1 https://github.com/DoosanRobotics/API-DRFL.git "$REPO/third_party/API-DRFL"
fi
echo "API-DRFL ready: $REPO/third_party/API-DRFL"
