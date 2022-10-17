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

from bluepy.btle import BTLEDisconnectError, BTLEGattError, DefaultDelegate, \
    Peripheral, ScanEntry, Scanner, UUID


class ScanDelegate(DefaultDelegate):

    def __init__(self):
        DefaultDelegate.__init__(self)

    def handleDiscovery(self, dev, isNewDev, isNewData):
        if isNewDev:
            print('Discovered device', dev.addr)
        elif isNewData:
            print('Received new data from', dev.addr)


class PeripheralDiviceInfo:

    def __init__(self, mac, name, addrType='random'):
        self.mac = mac
        self.name = name
        self.addrType = addrType


class BluetoothCore:
    """Providing BLE Central device interfaces."""

    def __init__(self):
        self.__scanner = Scanner()
        self.__scanner.withDelegate(ScanDelegate())
        self.__peripheral = Peripheral()
        self.__peripheral_list = []
        self.__connected = False
        self.__peripheral_name = ''

    def __del__(self):
        self.Disconnect()

    def Scan(self, sec=5.0) -> list:
        self.__peripheral_list.clear()
        devices = self.__scanner.scan(sec)
        for dev in devices:
            print('Device %s (%s), RSSI=%d dB' % (
                dev.addr, dev.getValueText(ScanEntry.COMPLETE_LOCAL_NAME), dev.rssi))
            if dev.getValueText(ScanEntry.COMPLETE_LOCAL_NAME) is not None:
                self.__peripheral_list.append(PeripheralDiviceInfo(
                    dev.addr, dev.getValueText(ScanEntry.COMPLETE_LOCAL_NAME),
                    dev.addrType))
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
        peripheral_inf = PeripheralDiviceInfo(mac, name, add_type)
        return self.__connect(peripheral_inf)

    def GetPeripheralInfo(self):
        if self.__connected:
            return (self.__peripheral.addr, self.__peripheral_name, self.__peripheral.addrType)
        return None

    def IsConnected(self):
        return self.__connected

    def Disconnect(self):
        self.__peripheral.disconnect()
        self.__connected = False
        self.__peripheral_name = ''
        print('bluetooth peripheral disconnected')

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

    def SetNotification(self, service, characteristic, enable=True):
        characteristic_handle = characteristic.getHandle()
        print('characteristic_handle:', characteristic_handle)
        descriptor_list = self.__peripheral.getDescriptors(characteristic_handle, service.hndEnd)
        descriptor_handle_for_notification = None
        for descriptor in descriptor_list:
            if descriptor.uuid == UUID(0x2902):
                print('found descriptor for notification')
                descriptor_handle_for_notification = descriptor.handle
                break
        if descriptor_handle_for_notification is None:
            print('Not found Client Characteristic Configuration')
            return None
        if enable:
            self.__peripheral.writeCharacteristic(
                descriptor_handle_for_notification, bytes([1, 0]))
        else:
            self.__peripheral.writeCharacteristic(
                descriptor_handle_for_notification, bytes([0, 0]))
        print('setNotification done')
        return characteristic_handle

    def SetNotificationByUUID(self, srv_uuid, char_uuid, enable: bool):
        service = self.GetService(srv_uuid)
        if service is None:
            return None
        characteristic = self.GetCharacteristic(service, char_uuid)
        if characteristic is None:
            return None
        return self.SetNotification(service, characteristic, enable)

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
            characteristic = self.GetCharacteristic(srv, characteristic_uuid)
            print('Start to write charicteristic')
            if characteristic is not None:
                if len(value) <= 20:
                    self.WriteCharacteristic(characteristic, value, withResponse)
                else:
                    packs = int(len(value) / 20)
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
                print('Writing complete')
                return True
        return False

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
            print(e, 'BLE device is disconnected unexpected!')
        except AttributeError as e:
            result = 3
            self.__connected = False
            self.__peripheral_name = ''
            print(e, 'BLE device is disconnected unexpected!')
        return result

    def GetPeripheralList(self):
        return self.__peripheral_list

    def __connect(self, peripheral_info: PeripheralDiviceInfo):
        try:
            self.__peripheral.connect(peripheral_info.mac, peripheral_info.addrType)
        except BTLEDisconnectError as e:
            print(e, peripheral_info.mac)
            return False
        self.__connected = True
        self.__peripheral_name = peripheral_info.name
        return True
