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

from bluepy.btle import BTLEDisconnectError, DefaultDelegate, Peripheral, ScanEntry, Scanner


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

    def Scan(self, sec=5.0):
        self.__peripheral_list.clear()
        devices = self.__scanner.scan(sec)
        for dev in devices:
            print('Device %s (%s), RSSI=%d dB' % (dev.addr, dev.addrType, dev.rssi))
        if dev.getValueText(ScanEntry.COMPLETE_LOCAL_NAME) is not None:
            self.__peripheral_list.append(PeripheralDiviceInfo(
                dev.addr, dev.getValueText(ScanEntry.COMPLETE_LOCAL_NAME), dev.addrType))
        return self.__peripheral_list

    def ConnectToBLEDeviceByName(self, name):
        if self.__connected:
            if name == self.__peripheral_name:
                return True
            else:
                self.Disconnect()
            for peripheral_info in self.__peripheral_list:
                if peripheral_info.name == name:
                    if not self.__connect(peripheral_info):
                        return False
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
                    if not self.__connect(peripheral_info):
                        return False
                    break
        return self.__connected

    def IsConnected(self):
        return self.__connected

    def Disconnect(self):
        self.__peripheral.disconnect()
        self.__connected = False

    def GetService(self, uuid):
        return self.__peripheral.getServiceByUUID(uuid)

    def GetCharacteristic(self, service, uuid):
        try:
            return service.getCharacteristics(uuid)[0]
        except IndexError:
            return None

    def GetCharacteristicWithoutService(self, uuid):
        try:
            return self.__peripheral.getCharacteristics(uuid=uuid)[0]
        except IndexError:
            return None

    def SetNotificationDelegate(self, notification):
        """Set a callback for notifications."""
        self.__peripheral.setDelegate(notification)

    def setNotification(self, service, characteristic, enable):
        characteristic_handle = characteristic.getHandle()
        print('characteristic_handle:', characteristic_handle)
        descriptor_list = self.__peripheral.getDescriptors(characteristic_handle, service.hndEnd)
        descriptor_handle_for_notification = None
        for descriptor in descriptor_list:
            if (descriptor.uuid == 0x2902):
                print('found descriptor for notification')
                descriptor_handle_for_notification = descriptor.handle
                break
        if descriptor_handle_for_notification is None:
            print('Not found ccn')
            return None
        if enable:
            self.__peripheral.writeCharacteristic(
                descriptor_handle_for_notification, bytes([1, 0]))
        else:
            self.__peripheral.writeCharacteristic(
                descriptor_handle_for_notification, bytes([0, 0]))
        print('setNotification done')
        return characteristic_handle

    def WriteCharacteristic(self, characteristic, value):
        characteristic.write(value)

    def ReadCharacteristic(self, characteristic):
        return characteristic.read()

    def __connect(self, peripheral_info):
        try:
            self.__peripheral.connect(peripheral_info.mac, peripheral_info.addrType)
        except BTLEDisconnectError:
            print('unable to connect to device', peripheral_info.mac)
            return False
        self.__connected = True
        self.__peripheral_name = peripheral_info.name
        return True
