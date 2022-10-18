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

from bluepy.btle import DefaultDelegate, UUID
from protocol.msg import BLEInfo
from protocol.srv import BLEConnect, BLEScan, GetUWBMacSessionID
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Joy
from std_msgs.msg import Bool

from . import bt_core
from . import yaml_parser


class BluetoothNode(Node, DefaultDelegate):
    """ROS2 node for cyberdog_bluetooth."""

    def __init__(self, node_name: str):
        Node.__init__(self, node_name)
        DefaultDelegate.__init__(self)
        self.__bt_central = bt_core.BluetoothCore()
        self.__map_mutex = threading.Lock()
        self.__character_handle_dic = {}
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
        self.__uart_received = False
        self.__uwb_connect_accepted = 3  # 3 default, 0 accepted, 1 rejected
        self.__uwb_disconnect_accepted = 3
        self.__multithread_callback_group = ReentrantCallbackGroup()
        self.__siglethread_callback_group = MutuallyExclusiveCallbackGroup()
        self.__scan_server = self.create_service(
            BLEScan, 'scan_bluetooth_devices', self.__scan_callback)
        self.__connect_server = self.create_service(
            BLEConnect, 'connect_bluetooth_device', self.__connect_callback,
            callback_group=self.__multithread_callback_group)
        self.__uwb_mac_session_id_client = self.create_client(
            GetUWBMacSessionID, 'get_uwb_mac_session_id',
            callback_group=self.__multithread_callback_group)
        self.__timeout_timer = self.create_timer(
            10.0, self.__timeoutCB, callback_group=self.__multithread_callback_group)
        self.__timeout_timer.cancel()
        self.__timeout_mutex = threading.Lock()
        self.__timeout = False
        self.__battery_volume_pub = self.create_publisher(BatteryState, 'band_battery', 2)
        self.__joystick_pub = self.create_publisher(Joy, 'bluetooth_joystick', 100)
        self.__notification_timer = self.create_timer(
            0.05, self.__notificationTimerCB, callback_group=self.__siglethread_callback_group)
        self.__notification_timer.cancel()
        self.__joystick_x = 0.0
        self.__joystick_y = 0.0
        self.__joystick_mutex = threading.Lock()
        self.__joystick_update = False
        self.__history_ble_list_file = '/home/mi/known_bles.yaml'
        self.__disconnect_unexpectedly_pub = self.create_publisher(
            Bool, 'bluetooth_disconnected_unexpected', 2)
        self.__current_connections_server = self.create_service(
            BLEScan, 'get_connected_bluetooth_info', self.__currentConnectionsCB)
        self.__connecting = False

    def __scan_callback(self, req, res):
        if abs(req.scan_seconds) < 0.001:  # get history device info
            history_info_list = self.__getHistoryConnectionInfo()
            if history_info_list is None:
                return res
            for dev_info in history_info_list:
                info = BLEInfo()
                info.mac = dev_info['mac']
                info.name = dev_info['name']
                info.addr_type = dev_info['addr_type']
                info.device_type = dev_info['device_type']
                res.device_info_list.append(info)
        elif not self.__bt_central.IsConnected():  # scan device info
            for dev_info in self.__bt_central.Scan(req.scan_seconds):
                info = BLEInfo()
                info.mac = dev_info.mac
                info.name = dev_info.name
                info.addr_type = dev_info.addrType
                res.device_info_list.append(info)
        else:
            for dev_info in self.__bt_central.GetPeripheralList():
                info = BLEInfo()
                info.mac = dev_info.mac
                info.name = dev_info.name
                info.addr_type = dev_info.addrType
                res.device_info_list.append(info)
        return res

    def __connect_callback(self, req, res):
        print('__connect_callback')
        if self.__connecting:
            print('is connecting!')
            res.result = 1
            return
        self.__connecting = True
        if req.selected_device.mac == '' or req.selected_device.mac is None:  # disconnect
            if not self.__bt_central.IsConnected():
                res.result = 3
            else:
                self.__notification_timer.cancel()
                self.__uwb_disconnect_accepted = 3
                self.__connectUWB(False)
                res.result = self.__waitForUWBResponse(False)
                if res.result != 0 and self.__bt_central.IsConnected():
                    self.__notification_timer.reset()
                    self.__connecting = False
                    return res
                self.__disconnectPeripheral()
                res.result = 0
        else:  # connect to device
            if self.__bt_central.IsConnected():
                connection_info = self.__bt_central.GetPeripheralInfo()
                if connection_info is not None:
                    mac, name, addr_type = connection_info
                    if mac == req.selected_device.mac:
                        res.result = 0
                        self.__connecting = False
                        return res
            self.__bt_central.SetNotificationDelegate(self)
            if self.__bt_central.ConnectToBLE(
                    req.selected_device.mac,
                    req.selected_device.name,
                    req.selected_device.addr_type):
                self.__getTagType()
                print('dev type', self.__connected_tag_type)
                if self.__connected_tag_type == 0:
                    print(req.selected_device.device_name, 'is not a cyberdog device!')
                    res.result = 1
                    self.__disconnectPeripheral()
                    self.__connecting = False
                    return res
                tx_handle = self.__bt_central.SetNotificationByUUID(  # TX char
                    self.__UART_service_uuid,
                    self.__TX_characteristic_uuid, True)
                if tx_handle is not None:
                    print('registering uart tx')
                    self.__registNotificationCallback(
                        tx_handle, self.__uartCB)
                if self.__connected_tag_type == 16:  # band
                    print("it 's a band")
                    battery_handle = self.__bt_central.SetNotificationByUUID(  # battery char
                        self.__battery_service_uuid,
                        self.__battery_level_characteristic_uuid, True)
                    if battery_handle is not None:
                        print('registering battery level')
                        self.__publishBatteryLevel(
                            self.__bt_central.ReadCharacteristicByHandle(battery_handle))
                        self.__registNotificationCallback(
                            battery_handle, self.__publishBatteryLevel)
                    joy_x_handle = self.__bt_central.SetNotificationByUUID(  # joystick x char
                        self.__remote_service_uuid,
                        self.__remote_x_characteristic_uuid, True)
                    if joy_x_handle is not None:
                        print('registering joyx')
                        self.__registNotificationCallback(
                            joy_x_handle, self.__joystickXCB)
                    joy_y_handle = self.__bt_central.SetNotificationByUUID(  # joystick y char
                        self.__remote_service_uuid,
                        self.__remote_y_characteristic_uuid, True)
                    if joy_y_handle is not None:
                        print('registering joyy')
                        self.__registNotificationCallback(
                            joy_y_handle, self.__joystickYCB)
                if self.__uwb_mac_session_id_client.wait_for_service(timeout_sec=3.0):
                    response = self.__uwb_mac_session_id_client.call(
                        GetUWBMacSessionID.Request())
                    # self.__bt_central.setMTU(512)  # set buffer size
                    self.__uwb_connect_accepted = 3
                    if self.__connectUWB(
                            True,
                            response.session_id,
                            response.master,
                            response.slave1, response.slave2, response.slave3, response.slave4):
                        res.result = self.__waitForUWBResponse(True)
                    else:
                        res.result = 2
                else:
                    res.result = 3
                if res.result != 0:
                    self.__disconnectPeripheral()
                else:
                    self.__notification_timer.reset()
                    new_connection = {
                        'mac': req.selected_device.mac,
                        'name': req.selected_device.name,
                        'addr_type': req.selected_device.addr_type,
                        'device_type': self.__connected_tag_type}
                    self.__updateHistoryFile(new_connection)
            else:
                res.result = 1
        self.__connecting = False
        return res

    def __registNotificationCallback(self, handle, callback):
        self.__map_mutex.acquire()
        self.__character_handle_dic[handle] = callback
        self.__map_mutex.release()

    def handleNotification(self, cHandle, data):
        print('receive data from characteristic', cHandle)
        self.__map_mutex.acquire()
        if cHandle in self.__character_handle_dic:
            self.__character_handle_dic[cHandle](data)
        self.__map_mutex.release()

    def __publishBatteryLevel(self, data):
        battery_level_float = int.from_bytes(data, 'little') / 100.0
        print('battery level is', battery_level_float)
        battery_msg = BatteryState()
        battery_msg.percentage = battery_level_float
        battery_msg.present = True
        self.__battery_volume_pub.publish(battery_msg)

    def __connectUWB(self, connect: bool, s_id=0, m=0, s1=0, s2=0, s3=0, s4=0):
        if not self.__bt_central.IsConnected():
            return not connect
        msg_header = b'\xaa\x55\x00\x01'
        device_type = self.__connected_tag_type.to_bytes(1, 'little')
        senser_frame_cmd = b'\x00\x00\x01' if connect else b'\x00\x00\x02'
        payload_length = (4 + 2 * 5).to_bytes(1, 'little') if connect else b'\x01'
        contents_to_send = bytearray()
        contents_to_send.extend(msg_header)
        contents_to_send.extend(device_type)
        contents_to_send.extend(senser_frame_cmd)
        contents_to_send.extend(payload_length)
        if connect:
            contents_to_send.extend(s_id.to_bytes(4, 'little'))
            contents_to_send.extend(m.to_bytes(2, 'little'))
            contents_to_send.extend(s1.to_bytes(2, 'little'))
            contents_to_send.extend(s2.to_bytes(2, 'little'))
            contents_to_send.extend(s3.to_bytes(2, 'little'))
            contents_to_send.extend(s4.to_bytes(2, 'little'))
        else:
            contents_to_send.extend(b'\x00')
        sum_int = 0
        for each_byte in bytes(contents_to_send):
            sum_int += each_byte
        contents_to_send.extend((sum_int % 0xFFFF).to_bytes(2, 'little'))
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
        print('receive uart data:', data)
        self.__uart_data_mutex.acquire()
        self.__uart_received = True
        if data[7] == 0x01:  # connect uwb response
            self.__uwb_connect_accepted = data[9]
        elif data[7] == 0x02:  # disconnect uwb response
            self.__uwb_disconnect_accepted = data[9]
        self.__uart_data_mutex.release()

    def __joystickXCB(self, data):
        self.__joystickCB(True, data)

    def __joystickYCB(self, data):
        self.__joystickCB(False, data)

    def __joystickCB(self, x_or_y: bool, data):
        self.__joystick_mutex.acquire()
        if x_or_y:
            self.__joystick_x = struct.unpack('f', data)[0]
        else:
            self.__joystick_y = struct.unpack('f', data)[0]
        self.__joystick_update = True
        self.__joystick_mutex.release()

    def __disconnectPeripheral(self):
        self.__notification_timer.cancel()
        self.__bt_central.Disconnect()
        self.__connected_tag_type = 0
        self.__map_mutex.acquire()
        self.__character_handle_dic.clear()
        self.__map_mutex.release()

    def __notificationTimerCB(self):
        notified = self.__bt_central.WaitForNotifications(0.1)
        if notified == 3:
            self.__notification_timer.cancel()
            self.__connected_tag_type = 0
            self.__map_mutex.acquire()
            self.__character_handle_dic.clear()
            self.__map_mutex.release()
            disconnect_msg = Bool()
            disconnect_msg.data = True
            self.__disconnect_unexpectedly_pub.publish(disconnect_msg)
            return
        if notified == 1:
            return
        self.__joystick_mutex.acquire()
        if self.__joystick_update:
            joy_msg = Joy()
            joy_msg.axes.append(self.__joystick_x)
            joy_msg.axes.append(self.__joystick_y)
            self.__joystick_pub.publish(joy_msg)
            self.__joystick_update = False
        self.__joystick_mutex.release()

    def __timeoutCB(self):
        self.__timeout_mutex.acquire()
        self.__timeout = True
        self.__timeout_mutex.release()
        self.__timeout_timer.cancel()

    def __waitForUWBResponse(self, connect: bool):
        got_uwb_response = False
        self.__timeout = False
        self.__timeout_timer.reset()
        result = 2
        while not got_uwb_response:
            self.__timeout_mutex.acquire()
            got_uwb_response = self.__timeout
            self.__timeout_mutex.release()
            wait_status = self.__bt_central.WaitForNotifications(0.1)
            if wait_status == 0:
                self.__uart_data_mutex.acquire()
                if self.__uart_received:
                    self.__uart_received = False
                    if connect:
                        if self.__uwb_connect_accepted != 3:
                            if self.__uwb_connect_accepted == 0:
                                result = 0
                            elif self.__uwb_connect_accepted == 1:
                                result = 2
                            self.__uwb_connect_accepted = 3
                            got_uwb_response = True
                    else:
                        if self.__uwb_disconnect_accepted != 3:
                            if self.__uwb_disconnect_accepted == 0:
                                result = 0
                            elif self.__uwb_disconnect_accepted == 1:
                                result = 2
                            self.__uwb_disconnect_accepted = 3
                            got_uwb_response = True
                self.__uart_data_mutex.release()
            elif wait_status == 3:
                break
        return result

    def __getHistoryConnectionInfo(self):
        return yaml_parser.YamlParser.GetYamlData(self.__history_ble_list_file)

    def __updateHistoryFile(self, new_ble_info):
        history_list = yaml_parser.YamlParser.GetYamlData(self.__history_ble_list_file)
        if history_list is None:
            history_list = []
        for info in history_list:
            if new_ble_info['mac'] == info['mac']:
                return True
        history_list.append(new_ble_info)
        return yaml_parser.YamlParser.GenerateYamlDoc(history_list, self.__history_ble_list_file)

    def __currentConnectionsCB(self, req, res):
        connection_info = self.__bt_central.GetPeripheralInfo()
        if connection_info is None:
            return res
        info = BLEInfo()
        info.mac, info.name, info.addr_type = connection_info
        info.device_type = self.__connected_tag_type
        res.device_info_list.append(info)
        return res
