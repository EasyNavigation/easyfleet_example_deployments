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

# Connects THIS shell to robot_1's Zenoh island, for introspection
# (`ros2 topic list`, `ros2 topic echo`, ...) -- see zenoh.md. Must be
# SOURCED, not executed, since it needs to export variables into your
# current shell rather than a throwaway subshell:
#
#   source set_robot_1.sh
#
# robot_1's own router (started by robot_1_launch.yaml or
# easynav_collaboration_launch.yaml) must already be running.

if ! (return 0 2>/dev/null); then
  echo "set_robot_1.sh must be sourced, not executed: 'source ${BASH_SOURCE[0]:-$0}'" >&2
  exit 1
fi

_easyfleet_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_SESSION_CONFIG_URI="${_easyfleet_script_dir}/../config/robot_1/zenoh_session.json5"
unset _easyfleet_script_dir

# `ros2 topic list`/`node list`/etc. talk to the `ros2 daemon`, a background
# process -- ONE per user/machine, shared by every terminal, not per-shell --
# that keeps a single rmw context alive for as long as it runs, fixed to
# whatever RMW_IMPLEMENTATION/ZENOH_SESSION_CONFIG_URI was set when it first
# started. It does not re-read any shell's environment on later `ros2`
# calls, and it is unrelated to the actual simulation (Gazebo, system_main,
# the Zenoh routers, ...), which never talks to it -- stopping it here is
# harmless to whatever else is running. Restarting it means THIS shell's
# next `ros2` command gets a fresh context using the variables just
# exported above.
ros2 daemon stop >/dev/null 2>&1

echo "This shell is now connected to robot_1's Zenoh island."
echo "  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "  ZENOH_SESSION_CONFIG_URI=${ZENOH_SESSION_CONFIG_URI}"
echo "If you also have another terminal open on a different island, add"
echo "--no-daemon to 'ros2 topic list'/'ros2 node list' etc. in both, or"
echo "they'll fight over the one shared daemon and one of them will see the"
echo "wrong island."
