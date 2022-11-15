#!/usr/bin/python3
#
# Copyright (c) 2022 Xiaomi Corporation
#
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

from protocol.action import Navigation
from protocol.srv import StopAlgoTask
from rclpy.action import ActionClient
from rclpy.node import Node


class UWBTracking:
    """Interfaces for tracking task."""

    def __init__(self, node: Node, multithread_callback_group):
        self.__stop_task_client = node.create_client(
            StopAlgoTask, 'stop_algo_task', callback_group=multithread_callback_group)
        self._tracking_action_client = ActionClient(
            node, Navigation, 'start_algo_task', callback_group=multithread_callback_group)

    def StopTracking(self):
        req = StopAlgoTask.Request()
        req.task_id = StopAlgoTask.Request.ALGO_TASK_UWB_TRACKING
        print('Calling stop uwb tracking.')
        self.__stop_task_client.call_async(req)

    def StartTracking(self):
        goal = Navigation.Goal()
        goal.nav_type = Navigation.Goal.NAVIGATION_TYPE_START_UWB_TRACKING
        print('Sending uwb tracking goal.')
        self._tracking_action_client.send_goal_async(goal)
