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

import struct
import threading
import time

from bluepy.btle import DefaultDelegate, UUID
from protocol.srv import BLEConnect, BLEScan, GetUWBMacSessionID
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Joy

from . import bt_core


class BluetoothNode(Node, DefaultDelegate):
    """ROS2 node for cyberdog_bluetooth."""

    def __init__(self, node_name: str):
        Node.__init__(self, node_name)
        DefaultDelegate.__init__(self)
        self.__bt_central = bt_core.BluetoothCore()
        self.__map_mutex = threading.Lock()
        self.__character_handle_dic = {}
        self.__bt_central.SetNotificationDelegate(self)
        self.__battery_service_uuid = UUID(0x180F)
        self.__battery_level_characteristic_uuid = UUID(0x2A19)
        self.__tag_info_service_uuid = UUID('6e400201-b5a3-f393-e0a9-e50e24dcca9e')
        self.__tag_type_characteristic_uuid = UUID('6e400202-b5a3-f393-e0a9-e50e24dcca9e')
        self.__UART_service_uuid = UUID('6e400001-b5a3-f393-e0a9-e50e24dcca9e')
        self.__RX_characteristic_uuid = UUID('6e400002-b5a3-f393-e0a9-e50e24dcca9e')
        self.__TX_characteristic_uuid = UUID('6e400003-b5a3-f393-e0a9-e50e24dcca9e')
        self.__remote_service_uuid = UUID('6e400101-b5a3-f393-e0a9-e50e24dcca9e')
        self.__remote_x_characteristic_uuid = UUID('6e400102-b5a3-f393-e0a9-e50e24dcca9e')
        self.__remote_y_characteristic_uuid = UUID('6e400103-b5a3-f393-e0a9-e50e24dcca9e')
        self.__connected_tag_type = 0  # 0 disconnected, 16 band, 17 dock
        self.__uart_data_mutex = threading.Lock()
        self.__uwb_connect_accepted = 3  # 3 default, 0 accepted, 1 rejected
        self.__uwb_disconnect_accepted = 3
        self.__callback_group = ReentrantCallbackGroup()
        self.__scan_server = self.create_service(
            BLEScan, 'scan_bluetooth_device', self.__scan_callback)
        self.__connect_server = self.create_service(
            BLEConnect, 'connect_bluetooth_device', self.__connect_callback,
            callback_group=self.__callback_group)
        self.__uwb_mac_session_id_client = self.create_client(
            GetUWBMacSessionID, 'get_uwb_mac_session_id',
            callback_group=self.__callback_group)
        self.__battery_volume_pub = self.create_publisher(BatteryState, 'band_battery', 2)
        self.__joystick_pub = self.create_publisher(Joy, 'bluetooth_joystick', 100)
        self.__joystick_x = 0.0
        self.__joystick_y = 0.0
        self.__joystick_mutex = threading.Lock()

    def __scan_callback(self, req, res):
        for dev_info in self.__bt_central.Scan(req.scan_seconds):
            res.device_name_list.append(dev_info.name)
        return res

    def __connect_callback(self, req, res):
        res.result = 2
        if req.device_name == '' or req.device_name is None:  # disconnect
            if not self.__bt_central.IsConnected():
                res.result = 3
            else:
                self.__uwb_disconnect_accepted = 3
                self.__connectUWB(False)
                got_uwb_response = False
                loop_count = 0
                while not got_uwb_response and loop_count < 60:
                    time.sleep(0.1)
                    loop_count += 1
                    self.__uart_data_mutex.acquire()
                    if self.__uwb_disconnect_accepted != 3:
                        if self.__uwb_disconnect_accepted == 0:
                            res.result = 0
                        elif self.__uwb_disconnect_accepted == 1:
                            res.result = 2
                        self.__uwb_disconnect_accepted = 3
                        got_uwb_response = True
                    self.__uart_data_mutex.release()
                if not got_uwb_response or res.result != 0:
                    return res
                self.__bt_central.Disconnect()
                self.__map_mutex.acquire()
                self.__character_handle_dic.clear()
                self.__map_mutex.release()
                res.result = 0
        else:  # connect to device
            if self.__bt_central.ConnectToBLEDeviceByName(req.device_name):
                self.__getTagType()
                if self.__connected_tag_type == 0:
                    print(req.device_name, 'is not a cyberdog device!')
                    res.result = 1
                    self.__bt_central.Disconnect()
                    return res
                tx_handle = self.__bt_central.SetNotificationByUUID(  # TX char
                    self.__UART_service_uuid,
                    self.__TX_characteristic_uuid, True)
                if tx_handle is not None:
                    self.__map_mutex.acquire()
                    self.__registNotificationCallback(
                        tx_handle, self.__uartCB)
                    self.__map_mutex.release()
                if self.__connected_tag_type == 16:  # band
                    battery_handle = self.__bt_central.SetNotificationByUUID(  # battery char
                        self.__battery_service_uuid,
                        self.__battery_level_characteristic_uuid, True)
                    if battery_handle is not None:
                        self.__publishBatteryLevel(
                            self.__bt_central.ReadCharacteristicByHandle(battery_handle))
                        self.__map_mutex.acquire()
                        self.__registNotificationCallback(
                            battery_handle, self.__publishBatteryLevel)
                        self.__map_mutex.release()
                    joy_x_handle = self.__bt_central.SetNotificationByUUID(  # joystick x char
                        self.__remote_service_uuid,
                        self.__remote_x_characteristic_uuid, True)
                    if joy_x_handle is not None:
                        self.__map_mutex.acquire()
                        self.__registNotificationCallback(
                            joy_x_handle, self.__joystickXCB)
                        self.__map_mutex.release()
                    joy_y_handle = self.__bt_central.SetNotificationByUUID(  # joystick y char
                        self.__remote_service_uuid,
                        self.__remote_y_characteristic_uuid, True)
                    if joy_y_handle is not None:
                        self.__map_mutex.acquire()
                        self.__registNotificationCallback(
                            joy_y_handle, self.__joystickYCB)
                        self.__map_mutex.release()
                if self.__uwb_mac_session_id_client.wait_for_service(timeout_sec=3.0):
                    response = self.__uwb_mac_session_id_client.call(
                        GetUWBMacSessionID.Request())
                    # time.sleep(0.5)
                    # self.__bt_central.setMTU(200)  # set buffer size
                    self.__uwb_connect_accepted = 3
                    if self.__connectUWB(
                            True,
                            response.session_id,
                            response.master,
                            response.slave1, response.slave2, response.slave3, response.slave4):
                        loop_count = 0
                        got_uwb_response = False
                        while not got_uwb_response and loop_count < 60:
                            time.sleep(0.1)
                            loop_count += 1
                            self.__uart_data_mutex.acquire()
                            if self.__uwb_connect_accepted != 3:
                                if self.__uwb_connect_accepted == 0:
                                    res.result = 0
                                elif self.__uwb_connect_accepted == 1:
                                    res.result = 2
                                self.__uwb_connect_accepted = 3
                                got_uwb_response = True
                            self.__uart_data_mutex.release()
                    else:
                        res.result = 2
                else:
                    res.result = 3
                if res.result != 0:
                    self.__bt_central.Disconnect()
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

    def __connectUWB(self, connect: bool, s_id=0, m=0, s1=0, s2=0, s3=0, s4=0):
        if not self.__bt_central.IsConnected():
            return not connect
        msg_header = b'\xaa\x00\x01'
        device_type = self.__connected_tag_type.to_bytes(1, 'little')
        frame_cmd = b'\x00\x01' if connect else b'\x00\x02'
        payload_length = (4 + 2 * 5).to_bytes(1, 'little') if connect else b'\x01'
        sum_bytes = (9 + 4 + 2 * 5).to_bytes(2, 'little') if connect else b'\x0a'
        contents_to_send = bytearray()
        contents_to_send.extend(msg_header)
        contents_to_send.extend(device_type)
        contents_to_send.extend(frame_cmd)
        contents_to_send.extend(payload_length)
        if connect:
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

    def __getTagType(self):
        tag_info_service = self.__bt_central.GetService(self.__tag_info_service_uuid)
        if tag_info_service is None:
            print('This device is not a cyberdog device!')
            return 0
        tag_type_char = self.__bt_central.GetCharacteristic(
            tag_info_service, self.__tag_type_characteristic_uuid)
        if tag_type_char is None:
            print('Cannot get device type!')
            return 0
        self.__connected_tag_type = int.from_bytes(
            self.__bt_central.ReadCharacteristic(tag_type_char), 'little')
        return self.__connected_tag_type

    def __uartCB(self, data):
        self.__uart_data_mutex.acquire()
        if data[7] == 0x01:  # connect uwb response
            self.__uwb_connect_accepted = data[9]
        elif data[7] == 0x02:  # disconnect uwb response
            self.__uwb_disconnect_accepted = data[9]
        self.__uart_data_mutex.release()

    def __joystickXCB(self, data):
        self.__joystickPublish(True, data)

    def __joystickYCB(self, data):
        self.__joystickPublish(False, data)

    def __joystickPublish(self, x_or_y: bool, data):
        self.__joystick_mutex.acquire()
        if x_or_y:
            self.__joystick_x = struct.unpack('f', data)[0]
        else:
            self.__joystick_y = struct.unpack('f', data)[0]
        joy_msg = Joy()
        joy_msg.axes.append(self.__joystick_x)
        joy_msg.axes.append(self.__joystick_y)
        self.__joystick_mutex.release()
        self.__joystick_pub.publish(joy_msg)
