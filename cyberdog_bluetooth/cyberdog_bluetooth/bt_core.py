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

from bluepy.btle import BTLEDisconnectError, BTLEGattError, BTLEInternalError, \
    DefaultDelegate, Peripheral, ScanEntry, Scanner, UUID


class ScanDelegate(DefaultDelegate):

    def __init__(self):
        DefaultDelegate.__init__(self)

    def handleDiscovery(self, dev, isNewDev, isNewData):
        if isNewDev:
            # print('Discovered device', dev.addr, 'type:', dev.addrType)
            pass
        elif isNewData:
            # print('Received new data from', dev.addr, 'type:', dev.addrType)
            pass


class PeripheralDiviceInfo:

    def __init__(self, mac, name, addrType='random', device_type=0):
        self.mac = mac
        self.name = name
        self.addrType = addrType
        self.device_type = device_type


class BluetoothCore:
    """Providing BLE Central device interfaces."""

    def __init__(self, logger):
        self.__logger = logger
        self.__scanner = Scanner()
        self.__scanner.withDelegate(ScanDelegate())
        self.__peripheral = Peripheral()
        self.__peripheral_list = []
        self.__connected = False
        self.__peripheral_name = ''

    def __del__(self):
        self.Disconnect()

    def Scan(self, sec=5) -> list:
        self.__peripheral_list.clear()
        devices = self.__scanner.scan(sec)
        for dev in devices:
            # print('Device %s (%s), RSSI=%d dB' % (
            #     dev.addr, dev.getValueText(ScanEntry.COMPLETE_LOCAL_NAME), dev.rssi))
            if dev.getValueText(ScanEntry.COMPLETE_LOCAL_NAME) is not None and\
                    dev.getValueText(ScanEntry.MANUFACTURER) is not None and\
                    dev.getValueText(ScanEntry.MANUFACTURER)[0: 4] == '8f03':
                self.__logger.info('Device %s (%s), RSSI=%d dB, is a Xiaomi Device' % (
                    dev.addr, dev.getValueText(ScanEntry.COMPLETE_LOCAL_NAME), dev.rssi))
                device_type = 0
                if dev.getValueText(ScanEntry.MANUFACTURER)[4: 8] == '105b':
                    device_type = 16
                elif dev.getValueText(ScanEntry.MANUFACTURER)[4: 8] == '115b':
                    device_type = 17
                self.__peripheral_list.append(PeripheralDiviceInfo(
                    dev.addr, dev.getValueText(ScanEntry.COMPLETE_LOCAL_NAME),
                    dev.addrType, device_type))
        return self.__peripheral_list

    def ConnectToBLEDeviceByName(self, name):
        if self.__connected:
            if name == self.__peripheral_name:
                return True
            else:
                self.Disconnect()
        for peripheral_info in self.__peripheral_list:
            if peripheral_info.name == name:
                self.__connect(peripheral_info)
                break
        return self.__connected

    def ConnectToBLEDeviceByMac(self, mac):
        if self.__connected:
            if mac == self.__peripheral.addr:
                return True
            else:
                self.Disconnect()
        for peripheral_info in self.__peripheral_list:
            if peripheral_info.mac == mac:
                self.__connect(peripheral_info)
                break
        return self.__connected

    def ConnectToBLE(self, mac, name, add_type):
        if self.__connected:
            if mac == self.__peripheral.addr:
                return True
            else:
                self.Disconnect()
        return self.__connect(PeripheralDiviceInfo(mac, name, add_type))

    def GetPeripheralInfo(self):
        if self.__connected:
            return (self.__peripheral.addr, self.__peripheral_name, self.__peripheral.addrType)
        return None

    def IsConnected(self):
        return self.__connected

    def Disconnect(self):
        try:
            self.__peripheral.disconnect()
        except BTLEDisconnectError as e:
            self.__logger.error(
                'BTLEDisconnectError: %s Disconnected unexpected while disconnecting!' % e)
        except AttributeError as e:
            self.__logger.error(
                'AttributeError: %s Exeption while disconnecting!' % e)
        finally:
            self.__connected = False
            self.__peripheral_name = ''
        self.__logger.info('bluetooth peripheral disconnected')

    def GetService(self, uuid):
        try:
            return self.__peripheral.getServiceByUUID(uuid)
        except BTLEGattError:
            return None

    def GetCharacteristic(self, service, uuid):
        try:
            return service.getCharacteristics(uuid)[0]
        except IndexError:
            return None

    def GetCharacteristicByUUID(self, srv_uuid, char_uuid):
        service = self.GetService(srv_uuid)
        if service is None:
            return None
        return self.GetCharacteristic(service, char_uuid)

    def SetNotificationDelegate(self, notification):
        """Set a callback for notifications."""
        self.__peripheral.setDelegate(notification)

    def SetNotification(self, service, characteristic, enable=True, indicate=False):
        characteristic_handle = characteristic.getHandle()
        self.__logger.info('characteristic_handle: %d' % characteristic_handle)
        descriptor_list = self.__peripheral.getDescriptors(characteristic_handle, service.hndEnd)
        if descriptor_list is None or len(descriptor_list) == 0:
            self.__logger.error('No descpriptors of this characteristic')
            return None
        if not self.__setNotficationByDescriptorList(descriptor_list, enable, indicate):
            return None
        return characteristic_handle

    def SetNotificationByUUID(self, srv_uuid, char_uuid, enable=True, indicate=False):
        service = self.GetService(srv_uuid)
        if service is None:
            self.__logger.error('No service found')
            return None
        characteristic = self.GetCharacteristic(service, char_uuid)
        if characteristic is None:
            self.__logger.error('No characteristic found')
            return None
        return self.SetNotification(service, characteristic, enable, indicate)

    def WriteCharacteristic(self, characteristic, value, withResponse=False):
        characteristic.write(value, withResponse)

    def ReadCharacteristic(self, characteristic):
        return characteristic.read()

    def SetMTU(self, size):
        self.__peripheral.setMTU(size)

    def Write(self, service_uuid, characteristic_uuid, value: bytes, withResponse=False):
        if not self.__connect:
            return False
        srv = self.GetService(service_uuid)
        if srv is not None:
            self.__logger.info('found service')
            characteristic = self.GetCharacteristic(srv, characteristic_uuid)
            if characteristic is not None:
                return self.__writeByCharacteristic(characteristic, value, withResponse)
            else:
                self.__logger.error('characteristic not found')
                return False
        self.__logger.error('service not found')
        return False

    def WriteByChracteristicUUID(self, characteristic_uuid, value: bytes, withResponse=False):
        if not self.__connect:
            return False
        characteristics = self.__peripheral.getCharacteristics(uuid=characteristic_uuid)
        if characteristics is not None and len(characteristics) != 0:
            return self.__writeByCharacteristic(characteristics[0], value, withResponse)
        self.__logger.error('Not found characteristic')
        return False

    def __writeByCharacteristic(self, characteristic, value: bytes, withResponse=False):
        self.__logger.info('Start to write characteristic')
        try:
            if len(value) <= 20:
                self.__logger.info('data is less than 20 bytes')
                self.WriteCharacteristic(characteristic, value, withResponse)
            else:
                packs = int(len(value) / 20)
                self.__logger.info(
                    'data is %d bytes, need to write %d packs' % (len(value), packs + 1))
                for i in range(0, packs):
                    print('Writing pack', i)
                    self.WriteCharacteristic(
                        characteristic,
                        value[i * 20: (i + 1) * 20],
                        withResponse)
                if len(value) % 20 != 0:
                    print('Writing last pack')
                    self.WriteCharacteristic(
                        characteristic,
                        value[packs * 20:],
                        withResponse)
            self.__logger.info('Writing complete')
        except BTLEDisconnectError as e:
            self.__connected = False
            self.__peripheral_name = ''
            self.__logger.error(
                'BTLEDisconnectError: %s Exeption while writing!' % e)
            return False
        except AttributeError as e:
            self.__connected = False
            self.__peripheral_name = ''
            self.__logger.error('AttributeError: %s Exeption while writing!' % e)
            return False
        except BTLEGattError as e:
            self.__connected = False
            self.__peripheral_name = ''
            self.__logger.error('BTLEGattError: %s Exeption while writing!' % e)
            return False
        return True

    def WriteCharacteristicByHandle(self, char_handel, value, withResponse=False):
        if self.__connected:
            self.__peripheral.writeCharacteristic(char_handel, value, withResponse)

    def ReadCharacteristicByHandle(self, char_handel):
        if self.__connected:
            return self.__peripheral.readCharacteristic(char_handel)
        return None

    def WaitForNotifications(self, sec):
        result = 0
        try:
            if self.__peripheral.waitForNotifications(sec):
                result = 0
            else:
                result = 1
        except BTLEDisconnectError as e:
            result = 3
            self.__connected = False
            self.__peripheral_name = ''
            self.__logger.error(
                'BTLEDisconnectError: %s Disconnected unexpected while waiting for notification!'
                % e)
        except AttributeError as e:
            result = 3
            self.__connected = False
            self.__peripheral_name = ''
            self.__logger.error(
                'AttributeError: %s Disconnected unexpected while waiting for notification!' % e)
        except BTLEGattError as e:
            result = 3
            self.__connected = False
            self.__peripheral_name = ''
            self.__logger.error(
                'BTLEGattError: %s Disconnected unexpected while waiting for notification!' % e)
        except BTLEInternalError as e:
            result = 3
            self.__connected = False
            self.__peripheral_name = ''
            self.__logger.error(
                'BTLEInternalError: %s Disconnected unexpected while waiting for notification!'
                % e)
        return result

    def GetPeripheralList(self):
        return self.__peripheral_list

    def GetCharacteristicList(self):
        return self.__peripheral.getCharacteristics()

    def SetNotificationByCharacteristicUUID(self, char_uuid: UUID, enable=True, indicate=False):
        characteristics = self.__peripheral.getCharacteristics(uuid=char_uuid)
        if len(characteristics) == 0:
            self.__logger.error('Not found characteristic')
            return None
        characteristic = characteristics[0]
        characteristic_handle = characteristic.getHandle()
        self.__logger.info('characteristic_handle: %d' % characteristic_handle)
        descriptor_list = self.__peripheral.getDescriptors(characteristic_handle)
        if descriptor_list is None or len(descriptor_list) == 0:
            self.__logger.error('No descpriptors of this characteristic')
            return None
        if not self.__setNotficationByDescriptorList(descriptor_list, enable, indicate):
            return None
        return characteristic_handle

    def __setNotficationByDescriptorList(self, descriptor_list: list, enable=True, indicate=False):
        descriptor_handle_for_notification = None
        for descriptor in descriptor_list:
            if descriptor.uuid == UUID(0x2902):
                self.__logger.info('found descriptor for notification')
                descriptor_handle_for_notification = descriptor.handle
                break
        if descriptor_handle_for_notification is None:
            self.__logger.error('Not found Client Characteristic Configuration')
            return False
        if enable:
            if not indicate:
                self.__peripheral.writeCharacteristic(
                    descriptor_handle_for_notification, bytes([1, 0]))
            else:
                self.__peripheral.writeCharacteristic(
                    descriptor_handle_for_notification, bytes([2, 0]))
        else:
            self.__peripheral.writeCharacteristic(
                descriptor_handle_for_notification, bytes([0, 0]))
        self.__logger.info('setNotification done!')
        return True

    def __connect(self, peripheral_info: PeripheralDiviceInfo):
        try:
            self.__peripheral.connect(peripheral_info.mac, peripheral_info.addrType)
        except BTLEDisconnectError as e:
            print(
                'BTLEDisconnectError:', e,
                'BLE device is disconnected unexpected while connecting')
            self.__logger.error(
                'BTLEDisconnectError: %s Disconnected unexpected while connecting!' % e)
            return False
        except ValueError as e:
            self.__logger.error(
                'ValueError: %s Disconnected unexpected while connecting!' % e)
            return False
        except BTLEInternalError as e:
            self.__logger.error(
                'BTLEInternalError: %s Disconnected unexpected while connecting!' % e)
            return False
        except BTLEGattError as e:
            self.__logger.error(
                'BTLEGattError: %s Disconnected unexpected while connecting!' % e)
            return False
        # self.__peripheral.setMTU(512)  # set buffer size
        self.__connected = True
        self.__peripheral_name = peripheral_info.name
        return True
