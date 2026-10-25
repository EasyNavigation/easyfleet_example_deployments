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

# Connects THIS shell to robot_2's DDS domain, for introspection
# (`ros2 topic list`, `ros2 topic echo`, ...) -- see fastdds.md. Must be
# SOURCED, not executed, since it needs to export a variable into your
# current shell rather than a throwaway subshell:
#
#   source set_robot_2.sh

if ! (return 0 2>/dev/null); then
  echo "set_robot_2.sh must be sourced, not executed: 'source ${BASH_SOURCE[0]:-$0}'" >&2
  exit 1
fi

export ROS_DOMAIN_ID=2

# See set_robot_1.sh for why the daemon needs restarting here.
ros2 daemon stop >/dev/null 2>&1

echo "This shell is now connected to robot_2's ROS_DOMAIN_ID=2."
echo "If you also have another terminal open on a different domain, add"
echo "--no-daemon to 'ros2 topic list'/'ros2 node list' etc. in both, or"
echo "they'll fight over the one shared daemon and one of them will see the"
echo "wrong domain."
