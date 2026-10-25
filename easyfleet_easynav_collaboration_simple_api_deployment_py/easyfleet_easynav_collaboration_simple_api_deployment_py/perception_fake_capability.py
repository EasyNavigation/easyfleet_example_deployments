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
Fake/mock implementation of the easyfleet_interfaces/Perception action, in Python.

A Python port of this deployment's own C++
perception_fake_capability.hpp/.cpp (a reference implementation of
easyfleet_core::PerceptionActionServerBase there, easyfleet_mission_manager_py's
ActionServerBase here): it does not run a real object detector. When
asked for the configured target class (a cat, "gato", by default) it
publishes feedback at a configurable rate (20 Hz by default) with 1 to 3
randomly placed detections, in both 2D and 3D, until the goal is
canceled, preempted or the node shuts down -- there is no other terminal
condition, mirroring how a continuous perception stream behaves. Goals
asking for any other class are rejected. `continuous`/`max_rate_hz` are
accepted but not honored by this mock: every goal streams forever at the
fixed `detection_rate_hz` parameter, since a mock has no notion of
"detect once and stop".
"""

import random
import time

from easyfleet_interfaces.action import Perception
from easyfleet_mission_manager_py import ActionServerBase, Capability, spin_capability
import rclpy
from vision_msgs.msg import (
    BoundingBox2D,
    BoundingBox3D,
    Detection2D,
    Detection3D,
    ObjectHypothesisWithPose,
)


def _requests_target_class(object_classes, target_class: str) -> bool:
    return any(target_class in requested.lower() for requested in object_classes)


class PerceptionFakeActionServer(ActionServerBase):
    """Reference mock implementation of the Perception action, in Python."""

    def __init__(self, node, action_name: str):
        super().__init__(node, Perception, action_name)
        self.target_class = node.declare_parameter(
            f'{action_name}.target_class', 'gato').value.lower()
        self.frame_id = node.declare_parameter(f'{action_name}.frame_id', 'camera_link').value
        self.detection_rate_hz = node.declare_parameter(
            f'{action_name}.detection_rate_hz', 20.0).value
        self._random = random.Random()

    def on_goal_received(self, goal) -> bool:
        return bool(goal.object_classes) and _requests_target_class(
            goal.object_classes, self.target_class)

    def _make_random_detections(self) -> Perception.Feedback:
        feedback = Perception.Feedback()
        feedback.detections_3d.header.frame_id = self.frame_id
        feedback.detections_2d.header.frame_id = self.frame_id

        for i in range(self._random.randint(1, 3)):
            score = self._random.uniform(0.6, 0.99)
            object_id = f'{self.target_class}_{i}'

            detection_3d = Detection3D()
            detection_3d.header.frame_id = self.frame_id
            detection_3d.id = object_id
            hypothesis_3d = ObjectHypothesisWithPose()
            hypothesis_3d.hypothesis.class_id = self.target_class
            hypothesis_3d.hypothesis.score = score
            hypothesis_3d.pose.pose.position.x = self._random.uniform(-3.0, 3.0)
            hypothesis_3d.pose.pose.position.y = self._random.uniform(-3.0, 3.0)
            hypothesis_3d.pose.pose.position.z = self._random.uniform(0.0, 1.0)
            detection_3d.results.append(hypothesis_3d)
            detection_3d.bbox = BoundingBox3D()
            detection_3d.bbox.center = hypothesis_3d.pose.pose
            detection_3d.bbox.size.x = 0.3
            detection_3d.bbox.size.y = 0.3
            detection_3d.bbox.size.z = 0.2
            feedback.detections_3d.detections.append(detection_3d)

            detection_2d = Detection2D()
            detection_2d.header.frame_id = self.frame_id
            detection_2d.id = object_id
            hypothesis_2d = ObjectHypothesisWithPose()
            hypothesis_2d.hypothesis.class_id = self.target_class
            hypothesis_2d.hypothesis.score = score
            detection_2d.results.append(hypothesis_2d)
            detection_2d.bbox = BoundingBox2D()
            detection_2d.bbox.center.position.x = self._random.uniform(0.0, 640.0)
            detection_2d.bbox.center.position.y = self._random.uniform(0.0, 480.0)
            detection_2d.bbox.size_x = 100.0
            detection_2d.bbox.size_y = 100.0
            feedback.detections_2d.detections.append(detection_2d)

        return feedback

    def on_execute(self, goal_handle):
        # continuous/max_rate_hz on the goal are accepted but not honored
        # by this mock: it always streams forever at self.detection_rate_hz
        # until canceled, preempted or shut down, regardless of what the
        # goal requested.
        period = 1.0 / self.detection_rate_hz
        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Perception.Result(error_code=Perception.Result.CANCELED)
            if self.is_preempt_requested():
                goal_handle.abort()
                return Perception.Result(
                    error_code=Perception.Result.ABORTED,
                    error_msg='Perception goal preempted by a newer goal.')
            if self.is_shutdown_requested():
                goal_handle.abort()
                return Perception.Result(
                    error_code=Perception.Result.ABORTED,
                    error_msg='Perception capability is shutting down.')

            goal_handle.publish_feedback(self._make_random_detections())
            time.sleep(period)


class PerceptionFakeCapability(Capability):
    """The "perception" capability, backed by the fake/mock action server."""

    def __init__(self, **kwargs):
        super().__init__('perception', PerceptionFakeActionServer, **kwargs)


def main(args=None):
    """Run the "perception" capability as a standalone node."""
    rclpy.init(args=args)
    spin_capability(PerceptionFakeCapability())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
