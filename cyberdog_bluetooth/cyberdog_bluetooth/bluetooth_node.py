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

import threading
import time

from bluepy.btle import DefaultDelegate, UUID
import bt_core
from protocol.srv import BLEConnect, BLEScan, GetUWBMacSessionID
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import BatteryState


class BluetoothNode(Node, DefaultDelegate):
    """ROS2 node for cyberdog_bluetooth."""

    def __init__(self, node_name: str):
        Node.__init__(node_name)
        DefaultDelegate.__init__()
        self.__bt_central = bt_core.BluetoothCore()
        self.__map_mutex = threading.Lock()
        self.__character_handle_dic = {}
        self.__bt_central.SetNotificationDelegate(self)
        self.__battery_service_uuid = UUID(0x180F)
        self.__battery_level_characteristic_uuid = UUID(0x2A19)
        self.__UART_service_uuid = UUID(0x0001)
        self.__RX_characteristic_uuid = UUID(0x0002)
        self.__TX_characteristic_uuid = UUID(0x0003)
        self.__remote_service_uuid = UUID(0x0101)
        self.__remote_x_characteristic_uuid = UUID(0x0102)
        self.__remote_y_characteristic_uuid = UUID(0x0103)
        self.__callback_group = ReentrantCallbackGroup()
        self.__scan_server = self.create_service(
            BLEScan, 'scan_bluetooth_device', self.__scan_callback)
        self.__connect_server = self.create_service(
            BLEConnect, 'connect_bluetooth_device', self.__connect_callback,
            callback_group=self.__callback_group)
        self.__uwb_mac_session_id_client = self.create_client(
            GetUWBMacSessionID, 'get_uwb_mac_session_id',
            callback_group=self.__callback_group)
        self.__battery_volume_pub = self.create_publisher(BatteryState, 'band_battery')
        # self.__joystic_cmd_timer = self.create_timer(0.1, )
        self.__joystic_x = 0.0
        self.__joystic_y = 0.0

    def __scan_callback(self, req, res):
        for dev_info in self.__bt_central.Scan(req.scan_seconds):
            res.device_name_list.append(dev_info.name)
        return res

    def __connect_callback(self, req, res):
        if req.device_name == '' or req.device_name is None:  # disconnect
            if not self.__bt_central.IsConnected():
                res.result = 3
            else:
                # disconnect UWB at first
                self.__bt_central.Disconnect()
                self.__map_mutex.acquire()
                self.__character_handle_dic.clear()
                self.__map_mutex.release()
                res.result = 0
        else:  # connect to device
            if self.__bt_central.ConnectToBLEDeviceByName(req.device_name):
                if self.__bt_central.GetConnectedDiveceType() == 16:
                    battery_handle = self.__bt_central.SetNotificationByUUID(
                        self.__battery_service_uuid,
                        self.__battery_level_characteristic_uuid, True)
                    if battery_handle is not None:
                        self.__publishBatteryLevel(
                            self.__bt_central.ReadCharacteristicByHandle(battery_handle))
                        self.__map_mutex.acquire()
                        self.__registNotificationCallback(
                            battery_handle, self.__publishBatteryLevel)
                        self.__map_mutex.release()
                if self.__uwb_mac_session_id_client.wait_for_service(timeout_sec=3.0):
                    response = self.__uwb_mac_session_id_client.call(
                        GetUWBMacSessionID.Request())
                    time.sleep(0.5)
                    # self.__bt_central.setMTU(200)  # set buffer size
                    if self.__connectUWB(
                            response.session_id,
                            response.master,
                            response.slave1, response.slave2, response.slave3, response.slave4):
                        res.result = 0
                    else:
                        res.result = 2
                else:
                    res.result = 3
            else:
                res.result = 1
        return res

    def __registNotificationCallback(self, handle, callback):
        self.__map_mutex.acquire()
        self.__character_handle_dic[handle] = callback
        self.__map_mutex.release()

    def handleNotification(self, cHandle, data):
        self.__map_mutex.acquire()
        if cHandle in self.__character_handle_dic:
            self.__character_handle_dic[cHandle](data)
        self.__map_mutex.release()

    def __publishBatteryLevel(self, data):
        battery_level_float = int.from_bytes(data, 'little') / 100.0
        print('battery level is', battery_level_float)
        battery_msg = BatteryState()
        battery_msg.percentage = battery_level_float
        self.__battery_volume_pub.publish(battery_msg)

    def __publishRemoteCMD(self, data):
        print('remote cmd is', data)

    def __joysticTimerCB(self):
        pass

    def __connectUWB(self, s_id, m, s1, s2, s3, s4):
        if not self.__bt_central.IsConnected():
            return False
        msg_header = b'\xaa\x00\x01'
        device_type = self.__bt_central.GetConnectedDiveceType().to_bytes(1, 'little')
        frame_cmd = b'\x00\x01'
        payload_length = (4 + 2 * 5).to_bytes(1, 'little')
        sum_bytes = (9 + 4 + 2 * 5).to_bytes(2, 'little')
        contents_to_send = bytearray()
        contents_to_send.extend(msg_header)
        contents_to_send.extend(device_type)
        contents_to_send.extend(frame_cmd)
        contents_to_send.extend(payload_length)
        contents_to_send.extend(s_id.to_bytes(4, 'little'))
        contents_to_send.extend(m.to_bytes(2, 'little'))
        contents_to_send.extend(s1.to_bytes(2, 'little'))
        contents_to_send.extend(s2.to_bytes(2, 'little'))
        contents_to_send.extend(s3.to_bytes(2, 'little'))
        contents_to_send.extend(s4.to_bytes(2, 'little'))
        contents_to_send.extend(sum_bytes)
        return self.__bt_central.Write(
            self.__UART_service_uuid,
            self.__RX_characteristic_uuid,
            bytes(contents_to_send), True)
