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

# Connects THIS shell to the central island (navigation_manager_node,
# easynav_collaboration_mission_node, fleet-wide rviz2, ...) -- see
# set_robot_1.sh for the full explanation. Must be SOURCED:
#
#   source set_central.sh
#
# From here you'll only see what each robot's own router ACL whitelists
# into the central island (/capabilities, /global_map, /global_routes,
# /clock, each robot's capability actions) -- not a given robot's internal
# topics (odom, scan, TF, ...). Connect to that robot's own island instead
# (set_robot_1.sh/set_robot_2.sh) to see those. See zenoh.md.
#
# The central router (started by easynav_collaboration_launch.yaml or
# simulation_launch.yaml) must already be running.

if ! (return 0 2>/dev/null); then
  echo "set_central.sh must be sourced, not executed: 'source ${BASH_SOURCE[0]:-$0}'" >&2
  exit 1
fi

_easyfleet_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_SESSION_CONFIG_URI="${_easyfleet_script_dir}/../config/zenoh_session_central.json5"
unset _easyfleet_script_dir

# See set_robot_1.sh for why this is needed: `ros2` CLI commands talk to a
# background daemon -- ONE per user/machine, shared by every terminal, and
# unrelated to the actual simulation -- that keeps one rmw context alive,
# fixed to whichever island was active when it first started. Restarting it
# here makes the next `ros2` command in THIS shell actually pick up the
# variables above.
ros2 daemon stop >/dev/null 2>&1

echo "This shell is now connected to the central Zenoh island."
echo "  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "  ZENOH_SESSION_CONFIG_URI=${ZENOH_SESSION_CONFIG_URI}"
echo "If you also have another terminal open on a different island, add"
echo "--no-daemon to 'ros2 topic list'/'ros2 node list' etc. in both, or"
echo "they'll fight over the one shared daemon and one of them will see the"
echo "wrong island."
