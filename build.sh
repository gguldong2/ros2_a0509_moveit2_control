#!/bin/bash
# Build a single DRFL program.
#   usage: ./build.sh <file.cpp> [DRCF_VERSION]   (default 2: controller GF02.x, use 3 for GF03.x)
# DRFL location: $DRFL_ROOT, else ./third_party/API-DRFL (scripts/setup_drfl.sh), else ~/doosan_ws/API-DRFL
set -e
REPO="$(cd "$(dirname "$0")" && pwd)"
DRFL="${DRFL_ROOT:-$REPO/third_party/API-DRFL}"
[ -f "$DRFL/include/DRFLEx.h" ] || DRFL="$HOME/doosan_ws/API-DRFL"
[ -f "$DRFL/include/DRFLEx.h" ] || { echo "API-DRFL not found. Run scripts/setup_drfl.sh"; exit 1; }

case "$(uname -m)" in
  x86_64)  ARCH=amd64 ;;
  aarch64) ARCH=arm64 ;;
  *) echo "unsupported arch: $(uname -m)"; exit 1 ;;
esac
LIB="$DRFL/library/Linux/64bits/$ARCH/$(lsb_release -rs)"
[ -f "$LIB/libDRFL.a" ] || { echo "no libDRFL.a for this Ubuntu version: $LIB"; exit 1; }
FOUNDATION=$(ls "$LIB"/libPocoFoundation.so.* | head -n1)
NET=$(ls "$LIB"/libPocoNet.so.* | head -n1)

SRC="$1"; VER="${2:-2}"
[ -f "$SRC" ] || { echo "usage: $0 <file.cpp> [DRCF_VERSION]"; exit 1; }
g++ -std=c++17 -O2 -DDRCF_VERSION="$VER" -I"$DRFL/include" "$SRC" -o "${SRC%.cpp}" \
    "$LIB/libDRFL.a" "$FOUNDATION" "$NET" -lpthread -ldl -Wl,-rpath,"$LIB"
echo "built ${SRC%.cpp}"
