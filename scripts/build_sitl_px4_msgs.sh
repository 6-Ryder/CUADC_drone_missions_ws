#!/usr/bin/env bash
# Build an isolated px4_msgs overlay from the PX4 source used by SITL.
# The real-flight src/px4_msgs package is deliberately left untouched.
set -eo pipefail

PROJ="$(cd "$(dirname "$0")/.." && pwd)"
PX4_DIR="${1:-/home/nvidia/PX4-Autopilot}"
TEMPLATE="$PROJ/src/px4_msgs"
BUILD_BASE="$PROJ/build/sitl_px4_msgs"
INSTALL_BASE="$PROJ/install/sitl_px4_msgs"
LOG_BASE="$PROJ/log/sitl_px4_msgs"

if [ ! -d "$PX4_DIR/msg/versioned" ] || [ ! -d "$TEMPLATE" ]; then
    echo "PX4 source or px4_msgs template missing" >&2
    exit 1
fi

if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "ROS 2 Humble is required" >&2
    exit 1
fi
source /opt/ros/humble/setup.bash
set -u

STAGE="$(mktemp -d /tmp/drone_sitl_px4_msgs.XXXXXX)"
cleanup() { rm -rf "$STAGE"; }
trap cleanup EXIT

cp -a "$TEMPLATE" "$STAGE/px4_msgs"
find "$STAGE/px4_msgs/msg" -maxdepth 1 -type f -name '*.msg' -delete
find "$STAGE/px4_msgs/srv" -maxdepth 1 -type f -name '*.srv' -delete
cp "$PX4_DIR/msg/"*.msg "$STAGE/px4_msgs/msg/"
cp "$PX4_DIR/msg/versioned/"*.msg "$STAGE/px4_msgs/msg/"
cp "$PX4_DIR/srv/"*.srv "$STAGE/px4_msgs/srv/"

colcon --log-base "$LOG_BASE" build \
    --base-paths "$STAGE/px4_msgs" \
    --packages-select px4_msgs \
    --build-base "$BUILD_BASE" \
    --install-base "$INSTALL_BASE" \
    --cmake-clean-cache \
    --cmake-force-configure
