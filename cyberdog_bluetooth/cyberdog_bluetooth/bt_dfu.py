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

import binascii
import os
import threading
import zipfile

from bluepy.btle import BTLEDisconnectError, BTLEGattError, BTLEInternalError,\
    DefaultDelegate, UUID

from . import bt_core


class Unzip:

    @staticmethod
    def unzipFile(zip_file: str, dst_dir: str):
        if zipfile.is_zipfile(zip_file):
            fz = zipfile.ZipFile(zip_file, 'r')
            file_name_list = fz.namelist()
            for file in file_name_list:
                fz.extract(file, dst_dir)
            return file_name_list
        else:
            print('This is not zip')
            return []


class BtDeviceFirmwareUpdate(DefaultDelegate):
    """Operations for bluetooth device firmware updating OTA."""

    def __init__(
            self, bt_core: bt_core.BluetoothCore, mac: str, zip_file: str,
            logger=None, progress=None):
        super().__init__()
        self.__logger = logger
        self.__bt_central = bt_core
        self.__dfu_mac = mac
        last_byte = self.__dfu_mac.split(':')[-1]
        last_byte_plus_one = hex(int(last_byte, 16) + 1)
        if len(last_byte_plus_one) == 3:
            last_byte_plus_one = '0' + last_byte_plus_one[-1]
        else:
            last_byte_plus_one = last_byte_plus_one[2:]
        self.__dfu_mac = self.__dfu_mac[0: -2] + last_byte_plus_one
        self.__logger.info('dfu_mac: %s' % self.__dfu_mac)
        self.__connected_to_dfu = False
        self.__zip_file = zip_file
        self.__init_package = ''
        self.__firmware_image = ''
        self.__unzip_dir = '/home/mi/.cyberdog/'
        self.__secure_dfu_service_uuid = UUID(0xFE59)  # useless
        self.__control_point_uuid = UUID('8ec90001-f315-4f60-9fb8-838830daea50')
        self.__dfu_packet_uuid = UUID('8ec90002-f315-4f60-9fb8-838830daea50')
        self.__notification_map_mutex = threading.Lock()
        self.__character_handle_dic = {}
        self.__ctrl_notify_event = threading.Event()
        self.__ctrl_notify_data = bytes()
        self.__connecting = False
        self.__pkt_payload_size = 20
        self.__pkt_receipt_interval = 0  # 10
        self.__trial_time = 5
        self.__progress_publisher = progress

    def ConnectToDFU(self):
        self.__connecting = True
        if self.__bt_central.IsConnected():
            self.__bt_central.Disconnect()
        try:
            self.__connected_to_dfu = self.__bt_central.ConnectToBLE(
                self.__dfu_mac, 'DFU', 'random')
            if self.__connected_to_dfu:
                self.__logger.info('DFU ble connected. Start to set Delegate.')
                self.__bt_central.SetNotificationDelegate(self)
                self.__logger.info('Turn on control point notification')
                ctrl_point_handle = self.__bt_central.SetNotificationByCharacteristicUUID(
                    self.__control_point_uuid, True)
                if ctrl_point_handle is None:
                    self.__connecting = False
                    return False
                self.__logger.info('registering control point handle: %d' % ctrl_point_handle)
                self.__registNotificationCallback(ctrl_point_handle, self.__ctrlPointCB)
        except BTLEDisconnectError as e:
            self.__logger.error(
                'BTLEDisconnectError: %s %s' % (
                    str(e),
                    'Disconnected unexpected while registering!'))
            self.__connected_to_dfu = False
        except AttributeError as e:
            self.__logger.error(
                'AttributeError: %s %s' % (
                    str(e),
                    'Disconnected unexpected while registering!'))
            self.__connected_to_dfu = False
        except BTLEInternalError as e:
            self.__logger.error(
                'BTLEInternalError: %s %s' % (
                    str(e),
                    'Disconnected unexpected while registering!'))
            self.__connected_to_dfu = False
        except BTLEGattError as e:
            self.__logger.error(
                'BTLEGattError: %s %s' % (
                    str(e),
                    'Disconnected unexpected while registering!'))
            self.__connected_to_dfu = False
        self.__connecting = False
        return self.__connected_to_dfu

    def DisconnectToDFU(self):
        self.__bt_central.Disconnect()
        with self.__notification_map_mutex:
            self.__character_handle_dic.clear()

    def UpdateFirmware(self):
        if not self.__connected_to_dfu:
            info = 'dfu device is not connected'
            self.__logger.error(info)
            msg = (6, 0.05, info)
            self.__progress_publisher(msg)
            return False
        info = 'start sending init package'
        self.__logger.info(info)
        msg = (5, 0.05, info)
        self.__progress_publisher(msg)
        if not self.__transferInitPackage():
            info = 'failed to transfer init package'
            self.__logger.error(info)
            msg = (6, 0.05, info)
            self.__progress_publisher(msg)
            return False
        info = 'start sending firmware image'
        self.__logger.info(info)
        msg = (7, 0.1, info)
        self.__progress_publisher(msg)
        if not self.__transferFirmwareImage():
            info = 'failed to transfer firmware image'
            self.__logger.error(info)
            msg = (8, 0.1, info)
            self.__progress_publisher(msg)
            return False
        return True

    def ProcessUpdate(self):
        if not self.UnzipFile():
            self.__logger.error('error while unzipping.')
            return False
        if not self.UpdateFirmware():
            self.__logger.error('error while updating')
            return False
        return True

    def AbortDFUMode(self):
        cmd = b'\x0c'  # Abort
        self.__logger.info('sending Abort: %s' % cmd)
        write_result = self.__bt_central.WriteByChracteristicUUID(
            self.__control_point_uuid,
            cmd, True)
        if not write_result:
            self.__logger.error('Error while writing Abort.')
            return False
        return True

    def UnzipFile(self):
        info = 'start zipping!'
        self.__logger.info(info)
        msg = (3, 0.0, info)
        self.__progress_publisher(msg)
        unzip_file_list = Unzip.unzipFile(self.__zip_file, self.__unzip_dir)
        if len(unzip_file_list) == 0:
            info = 'no zip file is found'
            self.__logger.error(info)
            msg = (4, 0.0, info)
            self.__progress_publisher(msg)
            return False
        for file_name in unzip_file_list:
            if file_name[-3:] == 'dat':
                self.__init_package = self.__unzip_dir + file_name
            elif file_name[-3:] == 'bin':
                self.__firmware_image = self.__unzip_dir + file_name
        return True

    def RemoveZipFile(self):
        if len(self.__zip_file):
            os.remove(self.__zip_file)

    def RemoveUnzipedFiles(self):
        if len(self.__init_package) != 0 and len(self.__firmware_image) != 0:
            os.remove(self.__init_package)
            os.remove(self.__firmware_image)

    def __writeCmdAndGetResp(self, cmd: bytes, descripton: str, timeout=2.0, trial_times=1):
        self.__logger.info('writing %s : %s' % (descripton, str(cmd)))
        write_result = self.__bt_central.WriteByChracteristicUUID(
            self.__control_point_uuid,
            cmd, True)
        if not write_result:
            self.__logger.error('Error while writing %s.' % descripton)
            return 3
        response_bytearray = bytearray()
        response_bytearray.extend(b'\x60')
        response_bytearray.extend(cmd[0: 1])
        response_bytearray.extend(b'\x01')
        result = 0
        while trial_times > 0:
            result = self.__waitForResponseSyn(bytes(response_bytearray), descripton, timeout)
            if result != 2:
                break
            trial_times -= 1
        return result

    def SetPRN(self, size=10):
        ctrl_bytearray = bytearray()
        ctrl_bytearray.extend(b'\x02')
        ctrl_bytearray.extend((size).to_bytes(4, 'little'))
        cmd = bytes(ctrl_bytearray)
        write_result = self.__writeCmdAndGetResp(cmd, 'set PRN', 2.0, 2)
        if write_result == 3:
            return False
        elif write_result == 0:
            self.__logger.info('set PRN seccessfully')

    def __transferInitPackage(self):
        self.SetPRN(self.__pkt_receipt_interval)
        if self.__init_package == '' or self.__init_package[-3:] != 'dat':
            self.__logger.warning('Init package is error')
            return False
        file_data = self.__readFile(self.__init_package)
        if len(file_data) == 0:
            self.__logger.warning('Init package size is 0')
            return False
        offset = 0
        crc = 0
        max_size = 0
        calculated_crc32 = binascii.crc32(file_data) & 0xFFFFFFFF
        file_offset = len(file_data)
        self.__logger.info('init pack size %d, crc %d' % (len(file_data), calculated_crc32))
        cmd = b'\x06\x01'  # select command
        write_result = self.__writeCmdAndGetResp(cmd, 'Select command', 2.0, 2)
        if write_result != 0 and write_result != 2:
            return False
        max_size = int.from_bytes(self.__ctrl_notify_data[3: 7], 'little')
        offset = int.from_bytes(self.__ctrl_notify_data[7: 11], 'little')
        crc = int.from_bytes(self.__ctrl_notify_data[11: 15], 'little')
        self.__logger.info('offset:%d crc:%d max_size:%d' % (offset, crc, max_size))
        trial_time = 3
        while (offset != file_offset or crc != calculated_crc32) and trial_time > 0:
            self.__logger.info('offset or crc is not the same')
            ctrl_bytearray = bytearray()
            ctrl_bytearray.extend(b'\x01\x01')
            ctrl_bytearray.extend(len(file_data).to_bytes(4, 'little'))
            cmd = bytes(ctrl_bytearray)  # create command
            write_result = self.__writeCmdAndGetResp(cmd, 'Create command', 2.0, 2)
            if write_result != 0 and write_result != 2:
                return False
            file_offset = 0
            offset = 0
            crc = 0
            self.__logger.info('start to write init pack')
            write_data_result = self.__bt_central.WriteByChracteristicUUID(
                self.__dfu_packet_uuid,
                file_data, False)
            if not write_data_result:
                return False
            file_offset = len(file_data)
            self.__logger.info('finish writing init pack')
            cmd = b'\x03'  # CRC
            write_result = 2
            while write_result == 2:
                write_result = self.__writeCmdAndGetResp(cmd, 'CRC', 2.0, 1)
            if write_result != 0:
                return False
            offset = int.from_bytes(self.__ctrl_notify_data[3: 7], 'little')
            crc = int.from_bytes(self.__ctrl_notify_data[7: 11], 'little')
            self.__logger.info('offset:%d crc:%d' % (offset, crc))
            trial_time -= 1
        if trial_time > 0 or (offset == file_offset and crc == calculated_crc32):
            cmd = b'\x04'  # Execute
            write_result = self.__writeCmdAndGetResp(cmd, 'Execute', 10.0, 3)
            if write_result != 0:
                return False
            self.__logger.info('finish executing init pack')
            return True
        self.__logger.error('fail to send init pack')
        return False

    def __waitForResponse(self, compare: bytes, operation: str, timeout=5.0):
        wait_flag = False
        if not self.__ctrl_notify_event.is_set():
            wait_flag = self.__ctrl_notify_event.wait(timeout)  # wait for response for 5s
        else:
            wait_flag = True
        self.__ctrl_notify_event.clear()
        if not wait_flag or self.__ctrl_notify_data[0: 3] != compare:
            if not wait_flag:
                self.__logger.warning('Timeout waiting for %s response.' % operation)
            else:
                self.__logger.info('response is not correct')
            return False
        return True

    def __waitForResponseSyn(self, compare: bytes, operation: str, timeout=5.0):
        wait_result = self.__bt_central.WaitForNotifications(timeout)
        if wait_result == 1:
            self.__logger.warning('Timeout waiting for %s response.' % operation)
            return 2
        elif wait_result == 3:
            self.__logger.error('Disconnected unexpectedly while waiting for notification')
            self.DisconnectToDFU()
            return 3
        else:
            if self.__ctrl_notify_data[0: 3] != compare:
                self.__logger.info(
                    'response is not correct: %s ' % str(self.__ctrl_notify_data[0: 3]))
                return 1
            else:
                return 0

    def __transferFirmwareImage(self):
        self.SetPRN(self.__pkt_receipt_interval)
        if self.__firmware_image == '' or self.__firmware_image[-3:] != 'bin':
            info = 'Firmware image is error'
            self.__logger.warning(info)
            msg = (8, 0.1, info)
            self.__progress_publisher(msg)
            return False
        file_data = self.__readFile(self.__firmware_image)
        if len(file_data) == 0:
            info = 'Firmware image size is 0'
            self.__logger.warning(info)
            msg = (8, 0.1, info)
            self.__progress_publisher(msg)
            return False
        offset = 0
        crc = 0
        max_size = 0
        calculated_crc32 = binascii.crc32(file_data) & 0xFFFFFFFF
        full_crc32 = calculated_crc32
        file_offset = 0
        self.__logger.info('firmware image size %d, crc %d' % (len(file_data), calculated_crc32))
        cmd = b'\x06\x02'  # select data
        write_result = self.__writeCmdAndGetResp(cmd, 'Select data', 10.0, 3)
        if write_result != 0:
            info = 'write command result is not correct'
            self.__logger.warning(info)
            msg = (8, 0.1, info)
            self.__progress_publisher(msg)
            return False
        max_size = int.from_bytes(self.__ctrl_notify_data[3: 7], 'little')
        offset = int.from_bytes(self.__ctrl_notify_data[7: 11], 'little')
        crc = int.from_bytes(self.__ctrl_notify_data[11: 15], 'little')
        self.__logger.info('offset:%d crc:%d max_size:%d' % (offset, crc, max_size))
        write_pack_per_obj = 0
        if max_size < len(file_data):
            write_pack_per_obj = int(max_size / self.__pkt_payload_size)
            if max_size / self.__pkt_payload_size > write_pack_per_obj:
                write_pack_per_obj += 1
        else:
            write_pack_per_obj = int(len(file_data) / self.__pkt_payload_size)
            if len(file_data) / self.__pkt_payload_size > write_pack_per_obj:
                write_pack_per_obj += 1
        self.__logger.info('it can be sent %d packs to max_size' % write_pack_per_obj)
        obj_sum = int(len(file_data) / max_size)
        if len(file_data) / max_size > obj_sum:
            obj_sum += 1
        self.__logger.info('total %d data objects' % obj_sum)
        crc_correct = True
        obj_index = 0
        while file_offset < len(file_data) or not crc_correct:
            reply_timeout = False
            if offset != len(file_data) or crc != full_crc32:  # sending is not finished
                ctrl_bytearray = bytearray()
                ctrl_bytearray.extend(b'\x01\x02')
                if len(file_data) - file_offset > max_size:
                    ctrl_bytearray.extend(max_size.to_bytes(4, 'little'))
                else:
                    ctrl_bytearray.extend(
                        (len(file_data) - file_offset).to_bytes(4, 'little'))
                cmd = bytes(ctrl_bytearray)  # create data
                write_result = self.__writeCmdAndGetResp(cmd, 'Create data', 2.0, 2)
                if write_result != 0 and write_result != 2:
                    return False
                self.__logger.info('start to write firmware image')
                this_obj_last_offset = min(file_offset + max_size, len(file_data))
                write_data_result = self.__bt_central.WriteByChracteristicUUID(
                    self.__dfu_packet_uuid,
                    file_data[file_offset: this_obj_last_offset], False)
                if not write_data_result:
                    info = 'write data result is not correct'
                    self.__logger.warning(info)
                    msg = (8, (obj_index * 1.0 / obj_sum) * 0.85 + 0.1, info)
                    self.__progress_publisher(msg)
                    return False
                file_offset = this_obj_last_offset
                self.__logger.info(
                    'writing firmware image process: %f' % (file_offset * 1.0 / len(file_data)))
                cmd = b'\x03'  # CRC
                write_result = self.__writeCmdAndGetResp(cmd, 'CRC', 2.0, 1)
                if write_result == 2:
                    reply_timeout = True
                elif write_result != 0:
                    info = 'write CRC result is not correct'
                    self.__logger.warning(info)
                    msg = (8, (obj_index * 1.0 / obj_sum) * 0.85 + 0.1, info)
                    self.__progress_publisher(msg)
                    return False
                offset = int.from_bytes(self.__ctrl_notify_data[3: 7], 'little')
                crc = int.from_bytes(self.__ctrl_notify_data[7: 11], 'little')
                self.__logger.info('offset:%d crc:%d' % (offset, crc))
            if reply_timeout or\
                    (offset == file_offset and crc == binascii.crc32(file_data[0: file_offset])):
                cmd = b'\x04'  # Execute
                write_result = self.__writeCmdAndGetResp(cmd, 'Execute', 2.0, 2)
                if write_result != 0 and write_result != 2:
                    info = 'write Execute result is not correct'
                    self.__logger.warning(info)
                    msg = (8, (obj_index * 1.0 / obj_sum) * 0.85 + 0.1, info)
                    self.__progress_publisher(msg)
                    return False
                obj_index += 1
                info = 'finish executing firmware image object %d / %d' % (obj_index, obj_sum)
                self.__logger.info(info)
                msg = (7, (obj_index * 1.0 / obj_sum) * 0.85 + 0.1, info)
                self.__progress_publisher(msg)
            else:
                crc_correct = False
                info = 'offset or crc is not correct, stop executing'
                self.__logger.warning(info)
                msg = (8, (obj_index * 1.0 / obj_sum) * 0.85 + 0.1, info)
                self.__progress_publisher(msg)
                return False
        info = 'finish executing firmware image'
        self.__logger.info(info)
        msg = (7, 0.95, info)
        self.__progress_publisher(msg)
        return True

    def __readFile(self, file_name: str):
        file = open(file_name, 'rb')
        file_data = file.read()
        file.close()
        return file_data

    def __registNotificationCallback(self, handle, callback):
        with self.__notification_map_mutex:
            self.__character_handle_dic[handle] = callback

    def handleNotification(self, cHandle, data):
        self.__logger.info('receive data from characteristic %d' % cHandle)
        with self.__notification_map_mutex:
            if cHandle in self.__character_handle_dic:
                self.__character_handle_dic[cHandle](data)

    def __ctrlPointCB(self, data):
        if len(data) == 0:
            self.__logger.warning('data is empty!')
            return
        self.__logger.info('receive data from ctrl point: %s' % data)
        self.__ctrl_notify_data = data
        self.__ctrl_notify_event.set()


class DFUFileChecker:

    def __init__(self, firmware_dir='/home/mi/.cyberdog/'):
        self.__directory = firmware_dir
        self.__band_prefix = 'BLE_UWB_TAG_BAND_APP_OTA_PACKAGE'
        self.__power_perfix = 'BLE_UWB_TAG_POWER_APP_OTA_PACKAGE'

    def Check(self, device_type: int, version: str):
        files = os.listdir(self.__directory)
        if len(files) == 0:
            return ''
        find_index = version.find('_')
        if find_index == -1:
            return ''
        date = version[find_index + 1: find_index + 9]
        result_list = []
        for file_name in files:
            if file_name[-3:] == 'zip':
                if device_type == 16 and self.__band_prefix in file_name:
                    result_list.append(file_name)
                elif device_type == 17 and self.__power_perfix in file_name:
                    result_list.append(file_name)
        if len(result_list) == 0:
            return ''
        max_date = 0
        max_candidate = ''
        try:
            for candidate in result_list:
                data_index = candidate.find('_20', 34) + 1
                candidate_date = int(candidate[data_index: data_index + 8])
                if candidate_date > max_date:
                    max_date = candidate_date
                    if max_candidate != '':
                        os.remove(self.__directory + max_candidate)
                    max_candidate = candidate
                else:
                    os.remove(self.__directory + candidate)
        except ValueError as e:
            print('ValueError', e)
            return ''
        if max_date > int(date):
            return self.__directory + max_candidate
        else:
            os.remove(self.__directory + max_candidate)
        return ''
