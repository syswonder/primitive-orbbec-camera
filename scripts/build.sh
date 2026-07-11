#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Build phase: rbnx codegen so atlas_bridge can import atlas_pb2 + lifecycle_pb2.
#
# Unlike realsense_camera_rbnx, there is no vendored colcon build here —
# orbbec_camera is apt-installed via ros-humble-orbbec-camera and lives
# under /opt/ros/humble/.
#
# Only rbnx codegen is needed. start.sh sources /opt/ros/humble/setup.bash
# before launching atlas_bridge.
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"
CLEAN="${RBNX_BUILD_CLEAN:-}"

if [[ "$CLEAN" == "1" ]]; then
    echo "[orbbec_camera/build] clean: removing rbnx-build/"
    rm -rf rbnx-build
fi
mkdir -p rbnx-build/data

FLAGS=(--out-dir "$PKG/rbnx-build/codegen" --mcp)
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)
echo "[orbbec_camera/build] rbnx codegen ${FLAGS[*]}"
rbnx codegen -p "$PKG" "${FLAGS[@]}"

touch "$PKG/rbnx-build/.rbnx-built"
echo "[orbbec_camera/build] done."
