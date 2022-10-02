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

from bluepy.btle import DefaultDelegate
import bt_core
from protocol.srv import BLEConnect, BLEScan, GetUWBMacSessionID
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node


class BluetoothNode(Node, DefaultDelegate):
    """ROS2 node for cyberdog_bluetooth."""

    def __init__(self, node_name: str):
        Node.__init__(node_name)
        DefaultDelegate.__init__()
        self.__bt_central = bt_core.BluetoothCore()
        self.__battery_service_uuid = 0x180F
        self.__battery_level_characteristic_uuid = 0x2A19
        self.__UART_service_uuid = 0x0001
        self.__RX_characteristic_uuid = 0x0002
        self.__TX_characteristic_uuid = 0x0003
        self.__remote_service_uuid = 0x0101
        self.__remote_x_characteristic_uuid = 0x0102
        self.__remote_y_characteristic_uuid = 0x0103
        self.__character_handle_dic = {}
        self.peripheral_type = 0  # 0 disconnected, 1 band, 2 dock
        self.__callback_group = ReentrantCallbackGroup()
        self.__scan_server = self.create_service(
            BLEScan, 'scan_bluetooth_device', self.__scan_callback)
        self.__connect_server = self.create_service(
            BLEConnect, 'connect_bluetooth_device', self.__connect_callback,
            callback_group=self.__callback_group)
        self.__uwb_mac_session_id_client = self.create_client(
            GetUWBMacSessionID, 'get_uwb_mac_session_id',
            callback_group=self.__callback_group)

    def __scan_callback(self, req, res):
        for dev_info in self.__bt_central.Scan(req.scan_seconds):
            res.device_name_list.append(dev_info.name)
        return res

    def __connect_callback(self, req, res):
        if req.device_name == '' or req.device_name is None:
            if not self.__bt_central.IsConnected():
                res.result = 3
            else:
                # disconnect UWB at first
                self.__bt_central.Disconnect()
                self.__character_handle_dic.clear()
                self.peripheral_type = 0
                res.result = 0
        else:
            if self.__bt_central.ConnectToBLEDeviceByName(req.device_name):
                if self.__uwb_mac_session_id_client.wait_for_service(timeout_sec=3.0):
                    # response = self.__uwb_mac_session_id_client.call(
                    #     GetUWBMacSessionID.Request())
                    # session_id = response.session_id
                    # master = response.master
                    # slave1 = response.slave1
                    # slave2 = response.slave2
                    # slave3 = response.slave3
                    # slave4 = response.slave4
                    # connect to UWB
                    # judge the type of peripheral_type
                    # regist callback functions
                    res.result = 0
                else:
                    res.result = 3
            else:
                res.result = 1
        return res

    def __registNotificationCallback(self, handle, callback):
        self.__character_handle_dic[handle] = callback

    def handleNotification(self, cHandle, data):
        self.__character_handle_dic[cHandle](data)

    def __publishBatteryLevel(self, data):
        print('battery level is', data)

    def __publishRemoteCMD(self, data):
        print('remote cmd is', data)
