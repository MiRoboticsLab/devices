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

import os
from queue import Queue
import struct
import threading
from time import sleep

from bluepy.btle import BTLEDisconnectError, BTLEGattError, BTLEInternalError,\
    DefaultDelegate, UUID
from nav2_msgs.srv import SaveMap
from protocol.msg import BLEDFUProgress, BLEInfo, MotionServoCmd
from protocol.srv import BLEConnect, BLEScan, GetBLEBatteryLevel, GetUWBMacSessionID
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Joy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from . import bt_core
from . import bt_dfu
from . import uwb_tracking
from . import yaml_parser


class BluetoothNode(Node, DefaultDelegate):
    """ROS2 node for cyberdog_bluetooth."""

    def __init__(self, node_name: str):
        Node.__init__(self, node_name)
        DefaultDelegate.__init__(self)
        self.__logger = self.get_logger()  # ROS2 logger
        self.__bt_central = bt_core.BluetoothCore()
        self.__notification_map_mutex = threading.Lock()
        self.__character_handle_dic = {}
        self.__battery_service_uuid = UUID(0x180F)
        self.__battery_level_characteristic_uuid = UUID(0x2A19)
        self.__GATT_service_uuid = UUID(0x180A)
        self.__software_version_characteristic_uuid = UUID(0x2A26)
        self.__tag_info_service_uuid = UUID('17b90201-3f76-7dba-4ad8-2f37edb7510b')
        self.__tag_type_characteristic_uuid = UUID('17b90202-3f76-7dba-4ad8-2f37edb7510b')
        self.__device_name_characteristic_uuid = UUID('17b90203-3f76-7dba-4ad8-2f37edb7510b')
        self.__UART_service_uuid = UUID('17b90001-3f76-7dba-4ad8-2f37edb7510b')
        self.__RX_characteristic_uuid = UUID('17b90002-3f76-7dba-4ad8-2f37edb7510b')
        self.__TX_characteristic_uuid = UUID('17b90003-3f76-7dba-4ad8-2f37edb7510b')
        self.__remote_service_uuid = UUID('17b90101-3f76-7dba-4ad8-2f37edb7510b')
        self.__remote_x_characteristic_uuid = UUID('17b90102-3f76-7dba-4ad8-2f37edb7510b')
        self.__remote_y_characteristic_uuid = UUID('17b90103-3f76-7dba-4ad8-2f37edb7510b')
        self.__dfu_service_uuid = UUID(0xFE59)
        self.__dfu_characteristic_uuid = UUID('8ec90003-f315-4f60-9fb8-838830daea50')
        self.__joy_x_char = None
        self.__joy_y_char = None
        self.__connected_tag_type = 0  # 0 disconnected, 16 band, 17 dock
        self.__firmware_version = ''
        self.__uart_data_mutex = threading.Lock()
        self.__uart_received = False
        self.__uwb_connect_accepted = 3  # 3 default, 0 accepted, 1 rejected
        self.__uwb_disconnect_accepted = 3
        self.__connected_mac = ''
        self.__multithread_callback_group = ReentrantCallbackGroup()
        self.__siglethread_callback_group = MutuallyExclusiveCallbackGroup()
        self.__scan_server = self.create_service(
            BLEScan, 'scan_bluetooth_devices', self.__scan_callback,
            callback_group=self.__multithread_callback_group)
        self.__scan_mutex = threading.Lock()
        self.__local_scan_result_list = []
        self.__connect_server = self.create_service(
            BLEConnect, 'connect_bluetooth_device', self.__connect_callback,
            callback_group=self.__multithread_callback_group)
        self.__uwb_mac_session_id_client = self.create_client(
            GetUWBMacSessionID, 'get_uwb_mac_session_id',
            callback_group=self.__multithread_callback_group)
        self.__response_timeout_timer = self.create_timer(
            10.0, self.__responseTimeoutCB, callback_group=self.__multithread_callback_group)
        self.__response_timeout_timer.cancel()
        self.__timeout = False
        self.__battery_volume_pub = self.create_publisher(BatteryState, 'band_battery', 2)
        self.__battery_level_float = 0.0
        self.__battery_srv = self.create_service(
            GetBLEBatteryLevel, 'ble_device_battery_level', self.__batteryLevelServerCB,
            callback_group=self.__multithread_callback_group)
        self.__joystick_pub = self.create_publisher(Joy, 'bluetooth_joystick', 100)
        self.__notification_thread = threading.Thread(target=self.__notificationgThreading)
        self.__joystick_x = 0.0
        self.__joystick_y = 0.0
        self.__joystick_mutex = threading.Lock()
        self.__joystick_update = False
        self.__motion_servo_cmd_pub = self.create_publisher(
            MotionServoCmd, 'motion_servo_cmd', 100)
        self.__remote_moving = False
        self.__history_ble_list_file = '/home/mi/.cyberdog/known_bles.yaml'
        history_dir = self.__history_ble_list_file[0: self.__history_ble_list_file.find(
            self.__history_ble_list_file.split('/')[-1])]
        folder_exist = os.path.exists(history_dir)
        if not folder_exist:
            os.mkdir(history_dir)
            os.chmod(history_dir, 7 * 8 * 8 + 7 * 8 + 7)
        self.__history_updated = True
        self.__history_connection_buffer = []
        self.__history_scan_intersection = []
        self.__disconnect_unexpectedly_pub = self.create_publisher(
            Bool, 'bluetooth_disconnected_unexpected', 2)
        self.__current_connections_server = self.create_service(
            BLEScan, 'get_connected_bluetooth_info', self.__currentConnectionsCB,
            callback_group=self.__multithread_callback_group)
        self.__firmware_version_server = self.create_service(
            Trigger, 'ble_device_firmware_version', self.__firmwareVersionServerCB,
            callback_group=self.__multithread_callback_group)
        self.__connecting = False
        self.__connect_timeout_timer = self.create_timer(
            5.0, self.__connectTimeoutCB, callback_group=self.__multithread_callback_group)
        self.__connect_timeout_timer.cancel()
        self.__change_name_server = self.create_service(
            SaveMap, 'change_ble_device_name', self.__changeBLEDeviceName,
            callback_group=self.__siglethread_callback_group)
        self.__poll_mutex = threading.Lock()
        self.__delete_history_server = self.create_service(
            SaveMap, 'delete_ble_devices_history', self.__deleteHistoryCB)
        self.__joy_pub_timer = self.create_timer(
            0.05, self.__joyPubTimerCB, callback_group=self.__siglethread_callback_group)
        self.__joy_pub_timer.cancel()
        self.__uwb_connection_signal_pub = self.create_publisher(Bool, 'uwb_connected', 2)
        self.__uwb_tracking = uwb_tracking.UWBTracking(self, self.__multithread_callback_group)
        self.__is_tracking = False
        self.__tread = ((303, 0.6, 1.0), (308, 1.0, 1.5), (305, 1.6, 2.0))
        self.__tread_index = 0
        self.__uart_ctrl_queue = Queue(5)
        self.__queue_mutex = threading.Lock()
        self.__auto_reconnect_timer = self.create_timer(
            30.0, self.__autoReconnect, callback_group=self.__multithread_callback_group)
        self.__enable_self_connection = True
        self.__app_status_sub = self.create_subscription(
            Bool, 'app_connection_state', self.__appConnectionCB, 1,
            callback_group=self.__siglethread_callback_group)
        self.__app_connected = False
        self.__dfu_notification_pub = self.create_publisher(
            String, 'ble_firmware_update_notification', 2)
        self.__dfu_process_service = self.create_service(
            Trigger, 'update_ble_firmware', self.__updateFirmwareCB,
            callback_group=self.__siglethread_callback_group)
        self.__dfu_file_checker = bt_dfu.DFUFileChecker('/home/mi/.cyberdog/')
        self.__firmware_candidate = ''
        self.__dfu_progress_publisher = self.create_publisher(
            BLEDFUProgress, 'ble_dfu_progress', 100)
        self.__dfu_timer = self.create_timer(
            0.12, self.__firmwareUpdateTimerCB,
            callback_group=self.__siglethread_callback_group)
        self.__dfu_timer.cancel()
        self.__bt_dfu_obj = None
        self.__dfu_processing = False
        self.__notification_thread.start()

    def __del__(self):
        self.__notification_thread.join()

    def __scan_callback(self, req, res):
        self.__logger.info('__scan_callback')
        if abs(req.scan_seconds) < 0.001:  # get history device info
            self.__logger.info('request history connections')
            history_info_list = self.__getHistoryConnectionInfo()
            if history_info_list is None or len(history_info_list) == 0:
                return res
            for dev_info in history_info_list:
                info = BLEInfo()
                info.mac = dev_info['mac']
                info.name = dev_info['name']
                info.addr_type = dev_info['addr_type']
                info.device_type = dev_info['device_type']
                info.firmware_version = dev_info['firmware_version']
                info.battery_level = 0.0
                res.device_info_list.append(info)
            self.__removeCurrentConnectionFromList(res.device_info_list)
        elif not self.__bt_central.IsConnected() and not self.__connecting:  # scan device info
            self.__logger.info('request scan')
            if self.__scan_mutex.acquire(blocking=False):
                self.__bt_central.Scan(req.scan_seconds)
                self.__getLatestScanResult()
                self.__removeHistoryFromList(self.__local_scan_result_list)
                self.__tryToReleaseMutex(self.__scan_mutex)
            else:  # got scan request while scanning
                self.__scan_mutex.acquire()
                self.__tryToReleaseMutex(self.__scan_mutex)
            res.device_info_list = self.__local_scan_result_list
        else:  # get latest scan result
            self.__logger.info('request scan, return latest scan result')
            self.__getLatestScanResult()
            self.__removeHistoryFromList(self.__local_scan_result_list)
            res.device_info_list = self.__local_scan_result_list
        return res

    def __getLatestScanResult(self):
        self.__local_scan_result_list.clear()
        for dev_info in self.__bt_central.GetPeripheralList():
            info = BLEInfo()
            info.mac = dev_info.mac
            info.name = dev_info.name
            info.addr_type = dev_info.addrType
            info.device_type = dev_info.device_type
            info.firmware_version = ''
            info.battery_level = 0.0
            self.__local_scan_result_list.append(info)
        return self.__local_scan_result_list

    def __connect_callback(self, req, res):
        self.__logger.info('__connect_callback')
        if self.__connecting:
            self.__logger.info('is connecting!')
            res.result = 1
            return res
        self.__connecting = True
        self.__scan_mutex.acquire()
        if req.selected_device.mac == '' or req.selected_device.mac is None:  # disconnect
            self.__logger.info('request disconnection')
            if not self.__bt_central.IsConnected():
                res.result = 3
            else:
                self.__joy_pub_timer.cancel()
                self.__uwb_disconnect_accepted = 3
                self.__connectUWB(False)
                self.__waitForUWBResponse(False)
                self.__disconnectPeripheral()
                res.result = 0
                self.__logger.info('disconnect complete')
        else:  # connect to device
            self.__logger.info('request connection')
            if self.__bt_central.IsConnected():
                connection_info = self.__bt_central.GetPeripheralInfo()
                if connection_info is not None:
                    mac, name, addr_type = connection_info
                    if mac == req.selected_device.mac:
                        res.result = 0
                        self.__tryToReleaseMutex(self.__scan_mutex)
                        self.__connecting = False
                        return res
                    else:
                        self.__uwb_disconnect_accepted = 3
                        self.__connectUWB(False)
                        res.result = self.__waitForUWBResponse(False)
                        self.__disconnectPeripheral()
            self.__connect_timeout_timer.reset()
            if self.__bt_central.ConnectToBLE(
                    req.selected_device.mac,
                    req.selected_device.name,
                    req.selected_device.addr_type):
                self.__bt_central.SetNotificationDelegate(self)
                try:
                    self.__getTagType()
                    self.__getTagFirmwareVersion()
                    self.__logger.info(
                        'device type %d firmware version %s' % (
                            self.__connected_tag_type, self.__firmware_version))
                    if self.__connected_tag_type == 0:
                        self.__logger.warning(
                            '%s is not a cyberdog device!' % req.selected_device.device_name)
                        res.result = 1
                        self.__disconnectPeripheral()
                        self.__tryToReleaseMutex(self.__scan_mutex)
                        self.__connecting = False
                        return res
                    tx_handle = self.__bt_central.SetNotificationByUUID(  # TX char
                        self.__UART_service_uuid,
                        self.__TX_characteristic_uuid, True)
                    if tx_handle is not None:
                        self.__logger.info('registering uart tx')
                        self.__registNotificationCallback(
                            tx_handle, self.__uartCB)
                    if self.__connected_tag_type == 16:  # band
                        self.__logger.info("it 's a band")
                        battery_handle = self.__bt_central.SetNotificationByUUID(  # battery char
                            self.__battery_service_uuid,
                            self.__battery_level_characteristic_uuid, True)
                        if battery_handle is not None:
                            self.__logger.info('registering battery level')
                            battery_first_time_reading =\
                                self.__bt_central.ReadCharacteristicByHandle(battery_handle)
                            if battery_first_time_reading is None:
                                res.result = 1
                                self.__disconnectPeripheral()
                                self.__tryToReleaseMutex(self.__scan_mutex)
                                self.__connecting = False
                                return res
                            self.__publishBatteryLevel(battery_first_time_reading)
                            self.__registNotificationCallback(
                                battery_handle, self.__publishBatteryLevel)
                        joy_x_handle = self.__bt_central.SetNotificationByUUID(  # joystick x char
                            self.__remote_service_uuid,
                            self.__remote_x_characteristic_uuid, True)
                        if joy_x_handle is not None:
                            self.__logger.info('registering joyx')
                            self.__registNotificationCallback(
                                joy_x_handle, self.__joystickXCB)
                        joy_y_handle = self.__bt_central.SetNotificationByUUID(  # joystick y char
                            self.__remote_service_uuid,
                            self.__remote_y_characteristic_uuid, True)
                        if joy_y_handle is not None:
                            self.__logger.info('registering joyy')
                            self.__registNotificationCallback(
                                joy_y_handle, self.__joystickYCB)
                    elif self.__connected_tag_type == 17:  # dock
                        self.__battery_level_float = 1.0
                        self.__joystick_x = 0.0
                        self.__joystick_y = 0.0
                except BTLEDisconnectError as e:
                    self.__logger.error(
                        'BTLEDisconnectError: %s Disconnected unexpected while registering!' % e)
                    res.result = 1
                except AttributeError as e:
                    self.__logger.error(
                        'AttributeError: %s Disconnected unexpected while registering!' % e)
                    res.result = 1
                except BTLEInternalError as e:
                    self.__logger.error(
                        'BTLEInternalError: %s Disconnected unexpected while registering!' % e)
                    res.result = 1
                except ValueError as e:
                    self.__logger.error(
                        'ValueError: %s Disconnected unexpected while registering!' % e)
                    res.result = 1
                except BTLEGattError as e:
                    self.__logger.error(
                        'BTLEGattError: %s Disconnected unexpected while registering!' % e)
                    res.result = 1
                if res.result == 1:
                    self.__disconnectPeripheral()
                    self.__tryToReleaseMutex(self.__scan_mutex)
                    self.__connecting = False
                    return res
                if self.__uwb_mac_session_id_client.wait_for_service(timeout_sec=3.0):
                    response = self.__uwb_mac_session_id_client.call(
                        GetUWBMacSessionID.Request())
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
                    self.__joy_pub_timer.reset()
                    new_connection = {
                        'mac': req.selected_device.mac,
                        'name': req.selected_device.name,
                        'addr_type': req.selected_device.addr_type,
                        'device_type': self.__connected_tag_type,
                        'firmware_version': self.__firmware_version}
                    self.__updateHistoryFile(new_connection)
                    self.__connected_mac = req.selected_device.mac
                    connection_signal = Bool()
                    connection_signal.data = True
                    self.__uwb_connection_signal_pub.publish(connection_signal)
                    if self.__app_connected and not self.__dfu_processing:
                        self.__checkAndPublishDFUNotification()
            else:
                res.result = 1
        self.__tryToReleaseMutex(self.__scan_mutex)
        self.__connecting = False
        return res

    def __registNotificationCallback(self, handle, callback):
        self.__notification_map_mutex.acquire()
        self.__character_handle_dic[handle] = callback
        self.__tryToReleaseMutex(self.__notification_map_mutex)

    def handleNotification(self, cHandle, data):
        self.get_logger().info(
            'receive data from characteristic %d' % cHandle, throttle_duration_sec=2.0)
        self.__notification_map_mutex.acquire()
        if cHandle in self.__character_handle_dic:
            self.__character_handle_dic[cHandle](data)
        self.__tryToReleaseMutex(self.__notification_map_mutex)

    def __publishBatteryLevel(self, data):
        if data is None:
            return
        self.__battery_level_float = int.from_bytes(data, 'little') / 100.0
        self.__logger.info('battery level is %f' % self.__battery_level_float)
        battery_msg = BatteryState()
        battery_msg.percentage = self.__battery_level_float
        battery_msg.present = True
        self.__battery_volume_pub.publish(battery_msg)

    def __connectUWB(self, connect: bool, s_id=0, m=0, s1=0, s2=0, s3=0, s4=0):
        if not self.__bt_central.IsConnected():
            return not connect
        if connect:
            payload = bytearray()
            payload.extend(s_id.to_bytes(4, 'little'))
            payload.extend(m.to_bytes(2, 'little'))
            payload.extend(s1.to_bytes(2, 'little'))
            payload.extend(s2.to_bytes(2, 'little'))
            payload.extend(s3.to_bytes(2, 'little'))
            payload.extend(s4.to_bytes(2, 'little'))
            return self.__uartCTL(b'\x01', bytes(payload), True)
        return self.__uartCTL(b'\x02', b'\x00', True)

    def __uartCTL(self, cmd: bytes, payload=None, response=True):
        msg_header = b'\xaa\x55\x00\x01'
        device_type = self.__connected_tag_type.to_bytes(1, 'little')
        senser_frame = b'\x00\x00'
        payload_length = 0 if payload is None else len(payload)
        contents_to_send = bytearray()
        contents_to_send.extend(msg_header)
        contents_to_send.extend(device_type)
        contents_to_send.extend(senser_frame)
        contents_to_send.extend(cmd)
        contents_to_send.extend(payload_length.to_bytes(1, 'little'))
        if payload is not None:
            contents_to_send.extend(payload)
        sum_int = 0
        for each_byte in bytes(contents_to_send):
            sum_int += each_byte
        contents_to_send.extend((sum_int % 0xFFFF).to_bytes(2, 'little'))
        return self.__bt_central.Write(
            self.__UART_service_uuid,
            self.__RX_characteristic_uuid,
            bytes(contents_to_send), response)

    def __getTagType(self):
        tag_info_service = self.__bt_central.GetService(self.__tag_info_service_uuid)
        if tag_info_service is None:
            self.__logger.warning('This device is not a cyberdog device!')
            return 0
        tag_type_char = self.__bt_central.GetCharacteristic(
            tag_info_service, self.__tag_type_characteristic_uuid)
        if tag_type_char is None:
            self.__logger.warning('Not able to get device type!')
            return 0
        self.__connected_tag_type = int.from_bytes(
            self.__bt_central.ReadCharacteristic(tag_type_char), 'little')
        return self.__connected_tag_type

    def __getTagFirmwareVersion(self):
        gatt_service = self.__bt_central.GetService(self.__GATT_service_uuid)
        if gatt_service is None:
            self.__logger.warning('This device has no GATT service!')
            return ''
        software_version_char = self.__bt_central.GetCharacteristic(
            gatt_service, self.__software_version_characteristic_uuid)
        if software_version_char is None:
            self.__logger.warning('Not able to get firmware version!')
            return ''
        self.__firmware_version = self.__bt_central.ReadCharacteristic(
            software_version_char).decode('UTF-8')
        return self.__firmware_version

    def __uartCB(self, data):
        self.__logger.info('receive uart data: %s' % data)
        self.__uart_data_mutex.acquire()
        self.__uart_received = True
        if data[7] == 0x01:  # connect uwb response
            self.__uwb_connect_accepted = data[9]
        elif data[7] == 0x02:  # disconnect uwb response
            self.__uwb_disconnect_accepted = data[9]
        elif data[7] == 0x04:  # uwb tracking
            task_status = self.__uwb_tracking.IsTrackingTaskActivated()
            if task_status == 11:
                self.__logger.info('Calling stop tracking service')
                self.__uwb_tracking.StopTracking()
                self.__is_tracking = False
            elif task_status == 101:
                self.__logger.info('Sending tracking goal to task action')
                self.__uwb_tracking.StartTracking()
                self.__is_tracking = True
        elif data[7] == 0x05:  # tread switching
            self.__tread_index = (self.__tread_index + 1) % 3
        self.__tryToReleaseMutex(self.__uart_data_mutex)

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
        self.__tryToReleaseMutex(self.__joystick_mutex)

    def __disconnectPeripheral(self):
        self.__poll_mutex.acquire()
        self.__joy_pub_timer.cancel()
        self.__notification_map_mutex.acquire()
        self.__character_handle_dic.clear()
        self.__tryToReleaseMutex(self.__notification_map_mutex)
        self.__bt_central.Disconnect()
        self.__connected_tag_type = 0
        self.__firmware_version = ''
        self.__tryToReleaseMutex(self.__poll_mutex)
        connection_signal = Bool()
        connection_signal.data = False
        self.__uwb_connection_signal_pub.publish(connection_signal)

    def __notificationTimerCB(self):
        notified = 0
        self.__poll_mutex.acquire()
        notified = self.__bt_central.WaitForNotifications(0.05)
        self.__tryToReleaseMutex(self.__poll_mutex)
        if notified == 3:
            self.__disconnectUnexpectedly()

    def __sendingUartCTL(self):
        self.__queue_mutex.acquire()
        if self.__uart_ctrl_queue.empty():
            self.__queue_mutex.release()
            return
        cmd, payload = self.__uart_ctrl_queue.get()
        self.__queue_mutex.release()
        if not self.__uartCTL(cmd, payload, True):
            self.__disconnectUnexpectedly()

    def __notificationgThreading(self):
        while rclpy.ok():
            if self.__connecting or not self.__bt_central.IsConnected():
                sleep(0.05)
            else:
                self.__sendingUartCTL()
                self.__notificationTimerCB()

    def __disconnectUnexpectedly(self):
        self.__tryToReleaseMutex(self.__poll_mutex)
        self.__disconnectPeripheral()
        disconnect_msg = Bool()
        disconnect_msg.data = True
        self.__disconnect_unexpectedly_pub.publish(disconnect_msg)

    def __responseTimeoutCB(self):
        self.__timeout = True
        self.__response_timeout_timer.cancel()

    def __waitForUWBResponse(self, connect: bool):
        got_uwb_response = False
        self.__timeout = False
        self.__response_timeout_timer.reset()
        result = 2
        while not got_uwb_response:
            got_uwb_response = self.__timeout
            self.__poll_mutex.acquire()
            wait_status = self.__bt_central.WaitForNotifications(0.01)
            self.__tryToReleaseMutex(self.__poll_mutex)
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
                self.__tryToReleaseMutex(self.__uart_data_mutex)
            elif wait_status == 3:
                break
        return result

    def __getHistoryConnectionInfo(self):
        if self.__history_updated:
            self.__history_connection_buffer = yaml_parser.YamlParser.GetYamlData(
                self.__history_ble_list_file)
            self.__history_updated = False
        return self.__history_connection_buffer

    def __updateHistoryFile(self, new_ble_info):
        history_list = yaml_parser.YamlParser.GetYamlData(self.__history_ble_list_file)
        if history_list is None:
            history_list = []
        i = 0
        found = 0
        for info in history_list:
            if new_ble_info['mac'] == info['mac']:
                if new_ble_info['name'] == info['name'] and\
                        new_ble_info['firmware_version'] == info['firmware_version']:
                    found = 2
                else:
                    found = 1
                break
            i += 1
        if found == 1:
            del history_list[i]
        elif found == 2:
            return True
        history_list.append(new_ble_info)
        self.__history_updated = True
        return yaml_parser.YamlParser.GenerateYamlDoc(history_list, self.__history_ble_list_file)

    def __currentConnectionsCB(self, req, res):
        connection_info = self.__bt_central.GetPeripheralInfo()
        if connection_info is None:
            return res
        info = BLEInfo()
        info.mac, info.name, info.addr_type = connection_info
        info.device_type = self.__connected_tag_type
        info.firmware_version = self.__firmware_version
        info.battery_level = self.__battery_level_float
        res.device_info_list.append(info)
        return res

    def __connectTimeoutCB(self):
        if not self.__bt_central.IsConnected():
            self.__logger.error('timeout and device is not able to connected')
            self.__bt_central.Disconnect()
        self.__connect_timeout_timer.cancel()

    def __batteryLevelServerCB(self, req, res):
        res.connected = self.__bt_central.IsConnected()
        if not res.connected:
            return res
        res.persentage = self.__battery_level_float
        return res

    def __firmwareVersionServerCB(self, req, res):
        res.success = self.__bt_central.IsConnected()
        if not res.success:
            return res
        res.message = self.__firmware_version
        return res

    def __changeBLEDeviceName(self, req, res):
        if self.__bt_central.Write(
                self.__tag_info_service_uuid, self.__device_name_characteristic_uuid,
                bytes(req.map_url, encoding='utf-8'), True):
            res.result = True
        else:
            res.result = False
        return res

    def __joyPollingCB(self, handel, x_or_y):
        self.__poll_mutex.acquire()
        data = self.__bt_central.ReadCharacteristicByHandle(handel)
        self.__tryToReleaseMutex(self.__poll_mutex)
        if x_or_y:
            self.__joystickXCB(data)
        else:
            self.__joystickYCB(data)

    def __joyPollingX(self, handel):
        self.__joyPollingCB(handel, True)

    def __joyPollingY(self, handel):
        self.__joyPollingCB(handel, False)

    def __deleteHistory(self, mac):
        history_info_list = self.__getHistoryConnectionInfo()
        if history_info_list is None or len(history_info_list) == 0:
            return False
        if mac == '':
            os.remove(self.__history_ble_list_file)
            return True
        i = 0
        found = False
        for dev_info in history_info_list:
            if dev_info['mac'] == mac:
                found = True
                break
            i += 1
        if found:
            del history_info_list[i]
            self.__history_updated = True
            return yaml_parser.YamlParser.GenerateYamlDoc(
                history_info_list, self.__history_ble_list_file)
        return False

    def __deleteHistoryCB(self, req, res):
        res.result = self.__deleteHistory(req.map_url)
        return res

    def __tryToReleaseMutex(self, mutex):
        try:
            mutex.release()
        except RuntimeError as re:
            self.__logger.error('RuntimeError: %s' % re)

    def __removeCurrentConnectionFromList(self, info_list):
        if not self.__bt_central.IsConnected():
            return
        connection_info = self.__bt_central.GetPeripheralInfo()
        if connection_info is None:
            return
        mac, name, addr_type = connection_info
        i = 0
        found = False
        for info in info_list:
            if info.mac == mac:
                found = True
                break
            i += 1
        if found:
            del info_list[i]

    def __removeHistoryFromList(self, info_list):
        history_list = self.__getHistoryConnectionInfo()
        if history_list is None or len(history_list) == 0:
            return
        i = 0
        found = []
        self.__history_scan_intersection.clear()
        for info in info_list:
            for info_h in history_list:
                if info.mac == info_h['mac']:
                    found.append(i)
                    self.__history_scan_intersection.append(info)
                    break
            i += 1
        j = 0
        if len(found) != 0:
            for numb in found:
                del info_list[numb - j]
                j += 1

    def __joyPubTimerCB(self):
        self.__joystick_mutex.acquire()
        if self.__joystick_update:
            joy_msg = Joy()
            joy_msg.axes.append(self.__joystick_x)
            joy_msg.axes.append(self.__joystick_y)
            self.__joystick_pub.publish(joy_msg)
            servo_cmd = MotionServoCmd()
            servo_cmd.motion_id = self.__tread[self.__tread_index][0]
            servo_cmd.cmd_type = 1
            servo_cmd.step_height.append(0.05)
            servo_cmd.step_height.append(0.05)
            servo_cmd.vel_des = [0.0, 0.0, 0.0]
            servo_cmd.cmd_source = 3
            if abs(self.__joystick_y) > 10:
                servo_cmd.vel_des[0] = self.__joystick_y / 50.0 * self.__tread[
                    self.__tread_index][1]
                self.__remote_moving = True
            if abs(self.__joystick_x) > 10:
                servo_cmd.vel_des[2] = -self.__joystick_x / 50.0 * self.__tread[
                    self.__tread_index][2]
                self.__remote_moving = True
            elif abs(self.__joystick_y) <= 10 and self.__remote_moving:
                servo_cmd.cmd_type = 2
                servo_cmd.step_height[0] = 0.0
                servo_cmd.step_height[1] = 0.0
            if self.__remote_moving:
                self.__motion_servo_cmd_pub.publish(servo_cmd)
            if servo_cmd.cmd_type == 2:
                self.__remote_moving = False
            self.__joystick_update = False
        self.__tryToReleaseMutex(self.__joystick_mutex)

    def __activateDFU(self):
        dfu_handle = self.__bt_central.SetNotificationByUUID(  # indicate
            self.__dfu_service_uuid,
            self.__dfu_characteristic_uuid, True, True)
        self.__logger.info('dfu handle: %d' % dfu_handle)
        self.__uwb_disconnect_accepted = 3
        result = self.__connectUWB(False)
        self.__logger.info('deactivate uwb')
        self.__waitForUWBResponse(False)
        self.__logger.info('start to write dfu_characteristic')
        result = self.__bt_central.Write(
            self.__dfu_service_uuid,
            self.__dfu_characteristic_uuid,
            b'\x01', True)
        self.__joy_pub_timer.cancel()
        sleep(1.0)
        self.__disconnectPeripheral()
        return result

    def __autoReconnect(self):
        if self.__bt_central.IsConnected() or\
                self.__connecting or not self.__enable_self_connection:
            return
        scan_req = BLEScan.Request()
        scan_res = BLEScan.Response()
        scan_req.scan_seconds = 3.0
        self.__scan_callback(scan_req, scan_res)
        if len(self.__history_scan_intersection) != 0:
            self.__logger.info(
                'find history device %s in scan result' % self.__history_scan_intersection[0].mac)
            connect_req = BLEConnect.Request()
            connect_res = BLEConnect.Response()
            connect_req.selected_device = self.__history_scan_intersection[0]
            self.__connect_callback(connect_req, connect_res)

    def __appConnectionCB(self, msg):
        if self.__app_connected == msg.data:
            return
        self.__app_connected = msg.data  # update status
        if self.__connecting:
            return
        if self.__app_connected and self.__bt_central.IsConnected() and not self.__dfu_processing:
            sleep(5)
            self.__checkAndPublishDFUNotification()

    def __checkAndPublishDFUNotification(self):
        self.__logger.info('Checking firmware updating.')
        self.__firmware_candidate = self.__dfu_file_checker.Check(
            self.__connected_tag_type, self.__firmware_version)
        if len(self.__firmware_candidate) > 36:
            msg = String()
            index = self.__firmware_candidate.find('APP_OTA_PACKAGE_V') + 16
            msg.data = self.__firmware_version + ' ' + self.__firmware_candidate[index: -4]
            self.__dfu_notification_pub.publish(msg)

    def __updateFirmwareCB(self, req, res):
        self.__connecting = True
        self.__dfu_processing = True
        self.__logger.info('activate dfu')
        self.__activateDFU()  # active dfu mode and disconnect current peripheral
        sleep(1.0)
        self.__logger.info('create dfu object')
        self.__bt_dfu_obj = bt_dfu.BtDeviceFirmwareUpdate(
            self.__bt_central, self.__connected_mac,
            self.__firmware_candidate,
            self.get_logger(), self.__publishProgress)
        self.__logger.info('start to connect to dfu device')
        if self.__bt_dfu_obj.ConnectToDFU():
            msg = 'dfu device connected'
            self.__logger.info(msg)
            res.success = True
            res.message = msg
            self.__dfu_timer.reset()
        else:
            msg = 'failed to connect to dfu'
            self.__logger.error(msg)
            res.success = False
            res.message = msg
            self.__bt_dfu_obj.DisconnectToDFU()
            self.__dfu_processing = False
            self.__connecting = False
        return res

    def __publishProgress(self, progress):
        msg = BLEDFUProgress()
        msg.status = progress[0]
        msg.progress = progress[1]
        msg.message = progress[2]
        self.__dfu_progress_publisher.publish(msg)

    def __firmwareUpdateTimerCB(self):
        self.__dfu_timer.cancel()
        if self.__bt_dfu_obj is None:
            self.__dfu_processing = False
            self.__connecting = False
            return
        if self.__bt_dfu_obj.ProcessUpdate():
            info = 'aborting dfu mode'
            self.__logger.info(info)
            progress = (9, 0.955, info)
            self.__publishProgress(progress)
            self.__bt_dfu_obj.AbortDFUMode()
            self.__logger.info('disconnect dfu')
            self.__bt_dfu_obj.DisconnectToDFU()
            info = 'wait for restarting'
            self.__logger.info(info)
            progress = (9, 0.96, info)
            self.__publishProgress(progress)
            sleep(5)
            info = 'reconnecting to new firmware device'
            self.__logger.info(info)
            progress = (9, 0.965, info)
            self.__publishProgress(progress)
            con_req = BLEConnect.Request()
            con_res = BLEConnect.Response()
            con_info = BLEInfo()
            con_info.addr_type = 'random'
            con_info.mac = self.__connected_mac
            con_req.selected_device = con_info
            self.__connecting = False
            self.__connect_callback(con_req, con_res)
            self.__connecting = True
            if con_res.result == 0:
                info = 'successfully upgraded'
                self.__logger.info(info)
                progress = (1, 1.0, 'successfully upgraded')
                self.__publishProgress(progress)
                self.__bt_dfu_obj.RemoveZipFile()
                self.__firmware_candidate = ''
            else:
                info = 'failed upgrading'
                self.__logger.error(info)
                progress = (10, 1.0, info)
                self.__publishProgress(progress)
            self.__bt_dfu_obj.RemoveUnzipedFiles()
        else:
            info = 'failed upgrading'
            self.__logger.error(info)
        self.__bt_dfu_obj = None
        self.__dfu_processing = False
        self.__connecting = False
