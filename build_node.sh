#!/usr/bin/env bash
# 用于构建一个NOde节点的脚本
set -e

WS_ROOT="${1:-$HOME/mk_nav2}"
PKG_NAME="${2:-frontier_explorer}"

cd "$WS_ROOT/src"

mkdir -p "$PKG_NAME"/{config,doc,include/"$PKG_NAME",launch,src,test}

touch "$PKG_NAME"/CHANGELOG.rst
touch "$PKG_NAME"/CMakeLists.txt
touch "$PKG_NAME"/package.xml

touch "$PKG_NAME"/include/"$PKG_NAME"/frontier_explorer_node.hpp
touch "$PKG_NAME"/src/frontier_explorer_node.cpp
touch "$PKG_NAME"/src/main.cpp

touch "$PKG_NAME"/config/frontier_explorer.yaml
touch "$PKG_NAME"/launch/frontier_explorer.launch.py

touch "$PKG_NAME"/test/.gitkeep
touch "$PKG_NAME"/doc/.gitkeep

echo "Created package skeleton: $WS_ROOT/src/$PKG_NAME"

EOF

chmod +x ~/mk_nav2/create_frontier_pkg.sh