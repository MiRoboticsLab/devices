#coding:utf-8
# Copyright 2016 Open Source Robotics Foundation, Inc.
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

import operator
import subprocess
from time import sleep

import rclpy
from rclpy.node import Node
from protocol.msg import WifiStatus
from protocol.srv import WifiConnect
from std_msgs.msg import Bool

RESULT_NO_SSID = 4
RESULT_ERR_PWD = 5
RESULT_OTHER = 6
RESULT_SUCCESS = 7
RESULT_INTERRUPT = 14
RESULT_TIMEOUT = 15
RESULT_HIDDEN_SSID = 16

# parsing the wlan connect/reconnect return value
nossid = 'No network with SSID'
errorpwd = 'Secrets were required, but not provided'
connected = 'successfully activated'
interrupt = 'The base network connection was interrupted'
activatefail = 'Connection activation failed'
timeout = 'Timeout'
hidden_ssid = 'hidden SSID'

def return_connect_status(output):
    if operator.contains(output, connected):
        return RESULT_SUCCESS
    elif operator.contains(output, errorpwd):
        return RESULT_ERR_PWD
    elif operator.contains(output, nossid):
        return RESULT_NO_SSID
    elif operator.contains(output, interrupt) or operator.contains(output, activatefail):
        return RESULT_INTERRUPT
    elif operator.contains(output, timeout):
        return RESULT_TIMEOUT
    elif operator.contains(output, hidden_ssid):
        return RESULT_HIDDEN_SSID
    else:
        return RESULT_OTHER


def runCommand(cmd):
    output = subprocess.Popen(
        cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    output.wait()  # wait for cmd done
    tmp = str(output.stdout.read(), encoding='utf-8')
    return tmp

def reconnect(ssid):
    cmd = 'sudo nmcli connection up "' + ssid + '"'
    print(cmd)
    return runCommand(cmd)


def getIP(if_name: str):
    output = subprocess.Popen('ifconfig ' + if_name,
        shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    output.wait()  # wait for cmd done
    tmp = str(output.stdout.read(), encoding='utf-8')
    tmps = tmp.split('\n')[1].split(' ')
    for ip in tmps:
        if ip.find('.') != -1:
            return ip
    return "0.0.0.0"


class CyberdogWifi(Node):

    def __init__(self):
        super().__init__('cyberdog_wifi')
        self.connected_ssid = ""
        self.get_wifi_rssi()
        self.srv_wifi = self.create_service(WifiConnect, 'connect_wifi', self.wifi_connect)
        self.pub_rssi = self.create_publisher(WifiStatus, 'wifi_status', 10)
        self.mode_sub = self.create_subscription(Bool, 'app_connection_state', self.switchMode ,1)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.logger = self.get_logger()
        self.connection_list = []
        self.app_connected = True
        init_msg = Bool()
        init_msg.data = False
        self.switchMode(init_msg)

    def get_connected_ssid(self):
        """get wifi ssid"""
        res = subprocess.Popen('nmcli device status | grep wlan0',
            shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        result = bytes.decode(res.stdout.read()).strip()
        if result != '' and not ('disconnected' in result):
            str_list = result.split()
            connected_ssid = str_list[3]
            str_index = 4
            while str_index < len(str_list):
                connected_ssid += " " + str_list[str_index]
                str_index += 1
            self.connected_ssid = connected_ssid
        else:
            self.connected_ssid = ''
        return res.stdout.read()

    def get_wifi_rssi(self):
        """get signal strength"""
        res = subprocess.Popen('nmcli device wifi | grep ^[*]',
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT)
        result = bytes.decode(res.stdout.read()).strip()
        rssi = 0
        if len(result) != 0 and ('Mbit/s' in result) and\
            (not (('Processing' in result) or ('WARNING' in result))):
            str_list = result.split()
            if str_list[-2].isdigit(): #No SECURITY
                rssi = int(str_list[-2])
            elif str_list[-3].isdigit(): #SECURITY只有一项时
                rssi = int(str_list[-3])
            elif str_list[-4].isdigit(): #SECURITY有两项时
                rssi = int(str_list[-4])
            elif str_list[-5].isdigit(): #SECURITY有三项时
                rssi = int(str_list[-5])
            ssid_last_index = 0
            try:
                ssid_last_index = str_list.index('Mbit/s') - 4
            except:
                return 200 #exception
            connected_ssid = str_list[1]
            str_index = 2
            while str_index <= ssid_last_index:
                connected_ssid += " " + str_list[str_index]
                str_index += 1
            self.connected_ssid = connected_ssid
        elif len(result) == 0:
            rssi = 0
            self.connected_ssid = ''
        else:
            rssi = 200 #exception
        return rssi

    def wifi_connect(self, request, response):
        """connect wifi callback"""
        if request.ssid.find('\'') != -1 or request.ssid.find('\"') != -1:
            self.logger.error('Don\'t use quotation marks in ssid!')
            response.result = RESULT_NO_SSID
            response.code = response.result + 1320
            return response
        elif request.pwd.find('\'') != -1 or request.pwd.find('\"') != -1:
            self.logger.error('Don\'t use quotation marks in password!')
            response.result = RESULT_ERR_PWD
            response.code = response.result + 1320
            return response
        pwd_log = ''
        for i in range(0, len(request.pwd)):
            pwd_log += '*'
        self.logger.info('request ssid: %s, request pwd: %s' % (request.ssid, pwd_log))
        if self.connected_ssid == request.ssid:
            response.result = RESULT_SUCCESS
            self.logger.info('ssid is the same!')
        else:
            response.result = RESULT_NO_SSID
            self.updateConnectionList()
            if request.ssid in self.connection_list:
                self.logger.info('ssid is in history list')
                runCommand('sudo nmcli connection delete "' + request.ssid + '"')
            trial_times = 0
            hidden = False
            while response.result != RESULT_SUCCESS and trial_times < 4:
                sleep(1.0)
                self.logger.info(
                    'Try to connect %s trial times: %d' % (request.ssid, trial_times))
                timeout = 15 - trial_times
                connect_res = self.nmcliConnectWifi(request.ssid, request.pwd, timeout, hidden)
                self.logger.info(connect_res)
                response.result = return_connect_status(connect_res)
                if response.result == RESULT_ERR_PWD:
                    self.logger.warning('password is error, stop connecting')
                    break
                elif response.result == RESULT_NO_SSID and trial_times < 2:
                    self.logger.info('Rescan wifi list')
                    self.rescanWifi(request.ssid)
                    sleep(3.0)
                elif response.result == RESULT_NO_SSID and trial_times < 3:
                    self.logger.warning('ssid not found, delete recorded connections')
                    self.deleteAllRecordedConnections()
                    sleep(2.0)
                elif response.result == RESULT_NO_SSID:
                    self.logger.warning('ssid not found')
                    break
                elif response.result == RESULT_OTHER or response.result == RESULT_INTERRUPT:
                    self.logger.warning('Not able to connect to ssid %s now' % request.ssid)
                    sleep(3.0)
                elif response.result == RESULT_HIDDEN_SSID:
                    self.logger.warning('Need to wait more time for connecting hidden ssid')
                    sleep(1.0)
                elif response.result == RESULT_TIMEOUT:
                    self.logger.warning('Command timeout')
                    break
                trial_times += 1
            self.logger.info('finish tries %d' % trial_times)
            if response.result == RESULT_SUCCESS:
                self.logger.info('successfully connected')
                self.connected_ssid = request.ssid
            self.logger.info('The response result is %d' % response.result)
            if response.result == RESULT_SUCCESS:
                response.code = 1300
            else:
                response.code = response.result + 1320
        return response

    def deleteAllRecordedConnections(self):
        for connection in self.connection_list:
            cmd = 'sudo nmcli connection delete "' + connection + '"'
            self.logger.info(cmd)
            runCommand(cmd)
    
    def timer_callback(self):
        """publish wifi status periodically"""
        msg = WifiStatus()
        msg.ip = getIP('wlan0')
        if msg.ip != '0.0.0.0':
            msg.strength = self.get_wifi_rssi()
            if msg.strength > 100: #exception
                return
            msg.ssid = self.connected_ssid
            if self.connected_ssid != '':
                msg.is_connected = True
            else:
                msg.is_connected = False
        else:
            msg.is_connected = False
            msg.ssid = self.connected_ssid = ''
            msg.strength = 0
        self.logger.info(
            'ssid: %s, ip: %s, strength: %d' % (msg.ssid, msg.ip, msg.strength),
            throttle_duration_sec=10.0)
        self.pub_rssi.publish(msg)

    def updateConnectionList(self):
        self.connection_list.clear()
        self.logger.info('update connection list')
        raw_str = runCommand('nmcli connection show | grep wifi')
        str_list = raw_str.split('\n')
        num = len(str_list) - 1
        index = 0
        while index < num:
            one_line_str_list = str_list[index].split()
            if len(one_line_str_list) < 1:
                continue
            ssid = one_line_str_list[0]
            i = 1
            while i < len(one_line_str_list) - 3:
                ssid += " " + one_line_str_list[i]
                i += 1
            self.connection_list.append(ssid)
            index += 1

    def setAutoconnect(self, ssid, autoconnect):
        activate = ''
        if autoconnect:
            activate = 'yes'
        else:
            activate = 'no'
        cmd = 'sudo nmcli connection modify "' + ssid + '" connection.autoconnect ' + activate
        self.logger.info(cmd)
        runCommand(cmd)

    def switchMode(self, msg):
        if self.app_connected == msg.data:
            return
        if msg.data:
            self.get_wifi_rssi()
        self.updateConnectionList()
        for ssid in self.connection_list:
            if (msg.data and not(self.connected_ssid in ssid)) or not msg.data:
                self.setAutoconnect(ssid, not msg.data)
        self.app_connected = msg.data

    # do a new wlan connect
    def nmcliConnectWifi(self, ssid, pwd, timeout=18, hidden=True):
        cmd = 'sudo nmcli --wait ' + str(timeout) + ' device wifi connect "'
        cmd += ssid
        if pwd != '':
            cmd += '" password "'
            cmd += pwd
        cmd += '"'
        if hidden:
            cmd += ' hidden yes'
        self.logger.info(cmd)
        return runCommand(cmd)

    def rescanWifi(self, ssid=''):
        cmd = 'sudo nmcli device wifi rescan'
        if ssid != '':
            cmd += ' ssid "' + ssid + '"'
        self.logger.info(cmd)
        runCommand(cmd)

    def __del__(self):
        self.destroy_service(self.srv_wifi)


def main(args=None):
    rclpy.init(args=args)

    wifi_node = CyberdogWifi()
    wifi_node.get_logger().info('cyberdog_wifi started.')
    rclpy.spin(wifi_node)

    # Destroy the service attached to the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wifi_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
