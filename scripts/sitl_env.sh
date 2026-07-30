#!/usr/bin/env bash
# Source the workspace with the PX4 SITL message definitions before real-flight v1.

SITL_ENV_PROJECT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SITL_ENV_OVERLAY="$SITL_ENV_PROJECT/install/sitl_px4_msgs"
SITL_ENV_PX4_MSGS="$SITL_ENV_OVERLAY/px4_msgs"

if [ ! -f /opt/ros/humble/setup.bash ] || [ ! -f "$SITL_ENV_PROJECT/install/setup.bash" ]; then
    echo "ROS 2 Humble or workspace install is missing" >&2
    return 1
fi

source /opt/ros/humble/setup.bash
source "$SITL_ENV_PROJECT/install/setup.bash"

if [ ! -f "$SITL_ENV_OVERLAY/setup.bash" ]; then
    "$SITL_ENV_PROJECT/scripts/build_sitl_px4_msgs.sh"
fi
source "$SITL_ENV_OVERLAY/setup.bash"

_sitl_prepend_path() {
    local variable="$1"
    local prefix="$2"
    local value="${!variable:-}"
    local entry
    local filtered=""
    local old_ifs="$IFS"

    IFS=:
    for entry in $value; do
        if [ -n "$entry" ] && [ "$entry" != "$prefix" ]; then
            filtered="${filtered:+$filtered:}$entry"
        fi
    done
    IFS="$old_ifs"
    export "$variable=$prefix${filtered:+:$filtered}"
}

# colcon does not reorder an already-sourced prefix.  Explicitly promote the
# isolated v4 package ahead of the workspace's real-flight v1 package.
_sitl_prepend_path AMENT_PREFIX_PATH "$SITL_ENV_PX4_MSGS"
_sitl_prepend_path CMAKE_PREFIX_PATH "$SITL_ENV_PX4_MSGS"
_sitl_prepend_path LD_LIBRARY_PATH "$SITL_ENV_PX4_MSGS/lib"
_sitl_prepend_path PYTHONPATH \
    "$SITL_ENV_PX4_MSGS/local/lib/python3.10/dist-packages"
_sitl_prepend_path COLCON_PREFIX_PATH "$SITL_ENV_OVERLAY"

unset SITL_ENV_PROJECT SITL_ENV_OVERLAY SITL_ENV_PX4_MSGS
