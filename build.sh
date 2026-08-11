#!/usr/bin/env bash
set -euo pipefail

# ------------------------------------------
# Usage:
# ./build.sh                         -> build whole workspace
# ./build.sh frontier_explorer        -> build frontier_explorer_nodes and its deps
# ./build.sh exploration              -> build robot_interfaces/frontier_explorer_*/bringup/task_manager
# ./build.sh util_package             -> build util_package and its deps
# BUILD_TYPE=Debug ./build.sh frontier_explorer
# ------------------------------------------

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$SCRIPT_DIR"
BUILD_TYPE="${BUILD_TYPE:-RelWithDebInfo}"

cd "$WORKSPACE_ROOT"

if [ -f /opt/ros/humble/setup.bash ]; then
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    set -u
fi

refresh_compile_commands() {
    if [ -f "$WORKSPACE_ROOT/build/compile_commands.json" ]; then
        ln -sf build/compile_commands.json "$WORKSPACE_ROOT/compile_commands.json"
    elif [ -f "$WORKSPACE_ROOT/build/frontier_explorer_nodes/compile_commands.json" ]; then
        ln -sf build/frontier_explorer_nodes/compile_commands.json "$WORKSPACE_ROOT/compile_commands.json"
    fi

    if [ -f "$WORKSPACE_ROOT/compile_commands.json" ]; then
        echo "compile_commands.json -> $(readlink "$WORKSPACE_ROOT/compile_commands.json" || true)"
    fi
}

repair_robot_interfaces_environment() {
    local generated_dsv="$WORKSPACE_ROOT/build/robot_interfaces/ament_cmake_environment_hooks/package.dsv"
    local installed_dsv="$WORKSPACE_ROOT/install/robot_interfaces/share/robot_interfaces/package.dsv"

    if [ -f "$generated_dsv" ] && [ -f "$installed_dsv" ]; then
        cp "$generated_dsv" "$installed_dsv"
    fi
}

build_all() {
    colcon build \
        --symlink-install \
        --cmake-args \
        -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
}

build_up_to() {
    colcon build \
        --packages-up-to "$@" \
        --symlink-install \
        --cmake-args \
        -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
}

build_select() {
    colcon build \
        --packages-select "$@" \
        --symlink-install \
        --cmake-args \
        -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
}

echo "Workspace root: $WORKSPACE_ROOT"
echo "Build type: $BUILD_TYPE"

if [ "$#" -eq 0 ] || [ "$1" = "all" ]; then
    echo "Building entire workspace..."
    build_all
elif [ "$1" = "exploration" ]; then
    echo "Building exploration runtime packages..."
    build_select robot_interfaces frontier_explorer_core frontier_explorer_nodes autonomousr_explorer_bringup task_manager
elif [ "$1" = "frontier" ] || [ "$1" = "frontier_explorer" ]; then
    echo "Building frontier_explorer_nodes and dependencies..."
    build_up_to frontier_explorer_nodes
else
    echo "Building packages and dependencies: $*"
    build_up_to "$@"
fi

repair_robot_interfaces_environment
refresh_compile_commands

if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
    set +u
    # shellcheck disable=SC1091
    source "$WORKSPACE_ROOT/install/setup.bash"
    set -u
fi

echo "Build complete!"
