#!/usr/bin/env bash
set -e  # 出错就退出

# ------------------------------------------
# Usage:
# ./build.sh                  -> build whole workspace
# ./build.sh frontier_explorer -> build util_package + frontier_explorer
# ./build.sh util_package      -> build only util_package
# ------------------------------------------

# 要构建的包，如果没传参数，就 build 所有
PACKAGES="$@"

WORKSPACE_ROOT="$(pwd)"

echo "Workspace root: $WORKSPACE_ROOT"
echo "Packages to build: ${PACKAGES:-ALL}"

# 1️⃣ 如果没指定包，就 build 整个 workspace
if [ -z "$PACKAGES" ]; then
    echo "Building entire workspace..."
    colcon build --symlink-install 
    echo "Sourcing workspace..."
    source install/setup.bash
    exit 0
fi

# 2️⃣ 如果指定了包，先判断是否依赖 util_package
if [[ "$PACKAGES" != "util_package" ]]; then
    echo "Building util_package first..."
    colcon build --packages-select util_package --symlink-install 
    echo "Sourcing workspace..."
    source install/setup.bash
fi

# 3️⃣ 构建指定包
echo "Building requested packages: $PACKAGES"
colcon build --packages-select $PACKAGES --symlink-install

echo "Sourcing workspace again..."
source install/setup.bash

echo "Build complete!"