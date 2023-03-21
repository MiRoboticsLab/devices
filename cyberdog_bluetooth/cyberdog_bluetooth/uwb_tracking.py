#!/usr/bin/python3
#
# Copyright (c) 2023 Xiaomi Corporation
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
from protocol.srv import AlgoTaskStatus, StopAlgoTask
from rclpy.action import ActionClient
from rclpy.node import Node


class UWBTracking:
    """Interfaces for tracking task."""

    def __init__(self, node: Node, multithread_callback_group):
        self.__stop_task_client = node.create_client(
            StopAlgoTask, 'stop_algo_task', callback_group=multithread_callback_group)
        self.__tracking_action_client = ActionClient(
            node, Navigation, 'start_algo_task', callback_group=multithread_callback_group)
        self.__tracking_activating = 999  # 101 for IDLE, 11 for uwb, 999 for unavailable
        self.__task_status_client = node.create_client(
            AlgoTaskStatus, 'algo_task_status', callback_group=multithread_callback_group)

    def StopTracking(self):
        if not self.__stop_task_client.wait_for_service(timeout_sec=3.0):
            print('stop_algo_task service is not available')
            return False
        req = StopAlgoTask.Request()
        req.task_id = StopAlgoTask.Request.ALGO_TASK_UWB_TRACKING
        print('Calling stop uwb tracking.')
        self.__stop_task_client.call_async(req)
        return True

    def StartTracking(self):
        if not self.__tracking_action_client.wait_for_server(timeout_sec=3.0):
            print('start_algo_task action is not available')
            return False
        goal = Navigation.Goal()
        goal.nav_type = Navigation.Goal.NAVIGATION_TYPE_START_UWB_TRACKING
        print('Sending uwb tracking goal.')
        self.__tracking_action_client.send_goal_async(goal)
        return True

    def IsTrackingTaskActivated(self):
        if self.__task_status_client.wait_for_service(timeout_sec=3.0):
            response = self.__task_status_client.call(
                AlgoTaskStatus.Request())
            self.__tracking_activating = response.status
        else:
            self.__tracking_activating = 999
        print('UWB tracking status is', self.__tracking_activating)
        return self.__tracking_activating
