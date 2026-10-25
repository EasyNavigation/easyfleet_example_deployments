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

"""
Fake/mock implementation of the easyfleet_interfaces/Manipulation action, in Python.

A Python port of this deployment's own C++
manipulation_fake_capability.hpp/.cpp (a reference implementation of
easyfleet_core::ManipulationActionServerBase there,
easyfleet_mission_manager_py's ActionServerBase here): it does not drive
a real manipulator or interact with real controllers. It simulates
trajectory execution progress over a configurable duration, publishing
feedback at a configurable rate, so that the rest of a multi-capability
system can be integrated and tested before a real manipulator stack is
wired in through its own ActionServerBase subclass.
"""

import time

from easyfleet_interfaces.action import Manipulation
from easyfleet_mission_manager_py import ActionServerBase, Capability, spin_capability
import rclpy


def _goal_target_is_empty(goal) -> bool:
    if goal.mode == Manipulation.Goal.MODE_JOINT_TARGET:
        return not goal.joint_target.name
    if goal.mode == Manipulation.Goal.MODE_POSE_TARGET:
        return not goal.pose_target.header.frame_id
    if goal.mode == Manipulation.Goal.MODE_NAMED_TASK:
        return not goal.named_task
    return True


class ManipulationFakeActionServer(ActionServerBase):
    """Reference mock implementation of the Manipulation action, in Python."""

    def __init__(self, node, action_name: str):
        super().__init__(node, Manipulation, action_name)
        self.mock_execution_duration_s = node.declare_parameter(
            f'{action_name}.mock_execution_duration', 3.0).value
        self.mock_feedback_period_s = node.declare_parameter(
            f'{action_name}.mock_feedback_period', 0.3).value

    def on_goal_received(self, goal) -> bool:
        return not _goal_target_is_empty(goal)

    def on_execute(self, goal_handle):
        goal = goal_handle.request
        feedback = Manipulation.Feedback()
        feedback.state = 'EXECUTING'
        feedback.current_joint_state = goal.joint_target

        num_steps = max(1, int(self.mock_execution_duration_s / self.mock_feedback_period_s))

        for i in range(num_steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Manipulation.Result(
                    error_code=Manipulation.Result.CANCELED,
                    error_msg='Manipulation canceled by client.')
            if self.is_preempt_requested():
                goal_handle.abort()
                return Manipulation.Result(
                    error_code=Manipulation.Result.ABORTED,
                    error_msg='Manipulation preempted by a newer goal.')
            if self.is_shutdown_requested():
                goal_handle.abort()
                return Manipulation.Result(
                    error_code=Manipulation.Result.ABORTED,
                    error_msg='Manipulation capability is shutting down.')

            time.sleep(self.mock_feedback_period_s)
            feedback.progress = (i + 1) / num_steps
            goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        return Manipulation.Result(
            error_code=Manipulation.Result.SUCCESS, final_joint_state=goal.joint_target)


class ManipulationFakeCapability(Capability):
    """The "manipulation" capability, backed by the fake/mock action server."""

    def __init__(self, **kwargs):
        super().__init__('manipulation', ManipulationFakeActionServer, **kwargs)


def main(args=None):
    """Run the "manipulation" capability as a standalone node."""
    rclpy.init(args=args)
    spin_capability(ManipulationFakeCapability())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
