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


RESULT_NO_SSID = 4
RESULT_ERR_PWD = 5
RESULT_OTHER = 6
RESULT_SUCCESS = 7
RESULT_INTERRUPT = 14
RESULT_TIMEOUT = 15


# parsing the wlan connect/reconnect return value
def return_connect_status(output):
    nossid = 'No network with SSID'
    errorpwd = 'Secrets were required, but not provided'
    connected = 'successfully activated'
    interrupt = 'The base network connection was interrupted'
    activatefail = 'Connection activation failed'
    timeout = 'Timeout expired'
    # global log
    # log.logger.info('output' + repr(output))
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
    else:
        return RESULT_OTHER


def runCommand(cmd):
    output = subprocess.Popen(
        cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    output.wait()  # wait for cmd done
    tmp = str(output.stdout.read(), encoding='utf-8')
    print(tmp)
    return tmp


# do a new wlan connect
def nmcliConnectWifi(ssid, pwd):
    # global log
    disconnect_cmd = "sudo nmcli device disconnect wlan0"
    runCommand(disconnect_cmd)
    sleep(1.0)
    cmd = "sudo nmcli device wifi connect '"
    cmd += ssid
    cmd += "' password '"
    cmd += pwd
    cmd += "'"
    return runCommand(cmd)

def reconnect(ssid):
    cmd = "sudo nmcli c up " + ssid
    return runCommand(cmd)


def getIP(if_name: str):
    cmd = 'ifconfig ' + if_name
    output = subprocess.Popen(
        cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
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
        self.get_connected_ssid()
        self.srv_wifi = self.create_service(WifiConnect, 'connect_wifi', self.wifi_connect)
        self.pub_rssi = self.create_publisher(WifiStatus, 'wifi_status', 0)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def get_connected_ssid(self):
        """获取wifi名称"""
        cmd = 'nmcli device status | grep " connected " | grep wlan0'
        res = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        result = bytes.decode(res.stdout.read()).strip()
        if len(result) > 4:
            str_list = result.split()
            connected_ssid = str_list[3]
            str_index = 4
            while str_index < len(str_list) - 1:
                connected_ssid += " " + str_list[str_index]
                str_index += 1
            self.connected_ssid = connected_ssid
        else:
            self.connected_ssid = ''
        return res.stdout.read()

    def get_wifi_rssi(self):
        """获取信号强度"""
        if len(self.connected_ssid) == 0:
            return 0
        cmd = 'nmcli device wifi | grep "' + self.connected_ssid +'"'
        res = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        result = bytes.decode(res.stdout.read()).strip()
        rssi = 0
        if len(result) != 0:
            str_list = result.split()
            if str_list[-3].isdigit(): #SECURITY只有一项时
                rssi = int(str_list[-3])
            elif str_list[-4].isdigit(): #SECURITY有两项时
                rssi = int(str_list[-4])
            elif str_list[-5].isdigit(): #SECURITY有三项时
                rssi = int(str_list[-3])
        return rssi

    def wifi_connect(self, request, response):
        """连接wifi的服务处理"""
        if self.connected_ssid == request.ssid:
            response.result = RESULT_SUCCESS
        else:
            response.result = RESULT_NO_SSID
            trial_times = 0
            while response.result != RESULT_SUCCESS and trial_times < 5:
                response.result = return_connect_status(
                    nmcliConnectWifi(request.ssid, request.pwd))
                trial_times += 1
            if response.result == RESULT_SUCCESS:
                self.connected_ssid = request.ssid
            elif len(self.connected_ssid) != 0:
                reconnect(self.connected_ssid)
        return response
    
    def timer_callback(self):
        """定期发布wifi状态"""
        msg = WifiStatus()
        msg.ip = getIP('wlan0')
        if msg.ip != '0.0.0.0':
            self.get_connected_ssid()
            if 'Processing' in self.connected_ssid:  #switching
                msg.ssid = self.connected_ssid = ''
                msg.strength = 0
                msg.is_connected = False
            else:
                msg.ssid = self.connected_ssid
                msg.strength = self.get_wifi_rssi()
                msg.is_connected = True
        else:
            msg.is_connected = False
            msg.ssid = self.connected_ssid = ''
            msg.strength = 0
        self.pub_rssi.publish(msg)


    def __del__(self):
        self.destroy_service(self.srv_wifi)


def main(args=None):
    rclpy.init(args=args)

    wifi_node = CyberdogWifi()
    rclpy.spin(wifi_node)

    # Destroy the service attached to the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wifi_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
