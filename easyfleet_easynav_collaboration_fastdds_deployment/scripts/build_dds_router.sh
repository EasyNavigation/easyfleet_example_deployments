#!/usr/bin/env bash
# Copyright 2026 Intelligent Robotics Lab
#
# This file is part of the project EasyFleet
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Bootstraps eProsima DDS Router from source. This deployment bridges each
# robot's ROS_DOMAIN_ID with the central island's using DDS Router (see
# fastdds.md at the workspace root for the full design discussion). DDS
# Router is not available as a binary package in any conda-forge/robostack
# channel and is not itself an ament/colcon package, so it can't just be a
# <depend> in package.xml -- it has to be built once, out-of-tree, from its
# upstream sources, before this deployment's launch files can find the
# `ddsrouter` executable they invoke.
#
# Usage: run once (from inside this workspace's ROS environment, so the
# already-installed ros-rolling-fastdds/fastcdr get found and reused rather
# than rebuilt):
#   ./build_dds_router.sh [install_prefix]
# Default install_prefix: $HOME/.local/dds_router
#
# Every module below is plain CMake (no ament/colcon), so each is
# configured/built/installed into the same prefix in dependency order, each
# one finding the previous ones via -DCMAKE_PREFIX_PATH. This exact sequence
# was validated from scratch on 2026-08-29 against
# ros-rolling-fastdds==3.6.1 (see fastdds.md) with zero source patches needed
# anywhere in the chain -- the only non-default flag required was linking
# pthread explicitly for the final tool (this toolchain's conda-forge glibc
# doesn't fold pthread into libc the way a recent system glibc does).

set -euo pipefail

INSTALL_PREFIX="${1:-$HOME/.local/dds_router}"
WORK_DIR="$(mktemp -d)"
trap 'rm -rf "$WORK_DIR"' EXIT

echo "Building eProsima DDS Router into: $INSTALL_PREFIX"
mkdir -p "$INSTALL_PREFIX"

build_module() {
  local src_dir="$1"
  shift
  echo "=== Building $(basename "$src_dir") ==="
  cmake -B "$src_dir/build" -S "$src_dir" \
    -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" \
    -DCMAKE_PREFIX_PATH="$INSTALL_PREFIX" \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTS=OFF \
    "$@"
  cmake --build "$src_dir/build" -j"$(nproc)"
  cmake --install "$src_dir/build"
}

cd "$WORK_DIR"
git clone --depth 1 --branch 1.x https://github.com/eProsima/dev-utils.git dev_utils
git clone --depth 1 --branch 1.x https://github.com/eProsima/DDS-Pipe.git ddspipe
git clone --depth 1 https://github.com/eProsima/DDS-Router.git ddsrouter

build_module "$WORK_DIR/dev_utils/cmake_utils"
build_module "$WORK_DIR/dev_utils/cpp_utils"
build_module "$WORK_DIR/ddspipe/ddspipe_core"
build_module "$WORK_DIR/ddspipe/ddspipe_participants"
build_module "$WORK_DIR/ddspipe/ddspipe_yaml"
build_module "$WORK_DIR/ddsrouter/ddsrouter_core"
build_module "$WORK_DIR/ddsrouter/ddsrouter_yaml"
build_module "$WORK_DIR/ddsrouter/tools/ddsrouter_tool" -DCMAKE_EXE_LINKER_FLAGS="-lpthread"

echo ""
echo "DDS Router installed at: $INSTALL_PREFIX/bin/ddsrouter"
if [ "$INSTALL_PREFIX" != "$HOME/.local/dds_router" ]; then
  echo "NOTE: this deployment's launch files hardcode \$HOME/.local/dds_router"
  echo "as the install location -- since you passed a different prefix, edit"
  echo "the 'cmd'/'LD_LIBRARY_PATH' entries in robot_1_launch.yaml and"
  echo "robot_2_launch.yaml to point at $INSTALL_PREFIX instead."
fi
