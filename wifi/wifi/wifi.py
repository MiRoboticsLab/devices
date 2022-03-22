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

import rclpy
from rclpy.node import Node

import protocol.msg._wifi as wifi_info

from protocol.srv import Wifi

STATUS_WIFI_GET_INFO = 1
STATUS_WIFI_CONNECT = 2
STATUS_WIFI_CONNECTING = 3
STATUS_WIFI_NO_SSID = 4
STATUS_WIFI_ERR_PWD = 5
STATUS_WIFI_OTHER = 6
STATUS_WIFI_SUCCESS = 7
STATUS_WIFI_INTERRUPT = 14
STATUS_WIFI_TIMEOUT = 15


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
        return STATUS_WIFI_SUCCESS
    elif operator.contains(output, errorpwd):
        return STATUS_WIFI_ERR_PWD
    elif operator.contains(output, nossid):
        return STATUS_WIFI_NO_SSID
    elif operator.contains(output, interrupt) or operator.contains(output, activatefail):
        return STATUS_WIFI_INTERRUPT
    elif operator.contains(output, timeout):
        return STATUS_WIFI_TIMEOUT
    else:
        return STATUS_WIFI_OTHER


# do a new wlan connect
def nmcliConnectWifi(ssid, pwd):
    # global log
    cmd = "sudo nmcli device wifi connect '"
    cmd += ssid
    cmd += "' password '"
    cmd += pwd
    cmd += "'"
    output = subprocess.Popen(
        cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    output.wait()  # wait for cmd done
    tmp = str(output.stdout.read(), encoding='utf-8')
    print(tmp)
    return tmp


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


class WifiNode(Node):

    def __init__(self):
        super().__init__('wifi')
        self.get_connected_ssid()
        self.srv_wifi = self.create_service(Wifi, 'wifi', self.wifi_connect)
        self.pub_rssi = self.create_publisher(wifi_info.Wifi, 'wifi_rssi', 0)
        timer_period = 0.3  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0
        self.cmd = 'iw wlan0 link |grep signal | awk -F " " '
        self.cmd += "'{ print $2 }'"

    def get_connected_ssid(self):
        cmd = 'nmcli device status | grep " connected " | grep "wlan0" | awk -F " " '
        cmd += "'{ print $4 }'"
        res = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        result = bytes.decode(res.stdout.read()).strip()
        if len(result) != 0:
            self.connected_ssid = result
        else:
            self.connected_ssid = ''
        return res.stdout.read()

    def get_wifi_rssi(self):
        if len(self.connected_ssid) == 0:
            return 0
        cmd = 'nmcli device wifi | grep " ' + self.connected_ssid +' " | awk -F " " '
        cmd += "'{ print $7 }'"
        res = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        result = bytes.decode(res.stdout.read()).strip()
        rssi = 100
        if result.isdigit():
            rssi = int(result)
        else:
            cmd = 'nmcli device wifi | grep " ' + self.connected_ssid +' " | awk -F " " '
            cmd += "'{ print $6 }'"
            res = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            result = bytes.decode(res.stdout.read()).strip()     
            if result.isdigit():
                rssi = int(result)       
        return rssi

    def wifi_connect(self, request, response):
        if self.connected_ssid == request.ssid:
            response.result = STATUS_WIFI_SUCCESS
        else: 
            response.result = return_connect_status(
                nmcliConnectWifi(request.ssid, request.pwd))
            if response.result == STATUS_WIFI_SUCCESS:
                self.connected_ssid = request.ssid
        return response
    
    def timer_callback(self):
        msg = wifi_info.Wifi()
        msg.ssid = self.connected_ssid
        if len(msg.ssid) == 0:
            msg.is_connected = False
        else:
            msg.is_connected = True
        msg.strength = self.get_wifi_rssi()
        msg.ip = getIP('wlan0')
        self.pub_rssi.publish(msg)

    def __del__(self):
        self.destroy_service(self.srv_wifi)
        self.destroy_service(self.srv_ip)


def main(args=None):
    global g_node
    rclpy.init(args=args)

    wifi_node = WifiNode()
    rclpy.spin(wifi_node)

    # Destroy the service attached to the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wifi_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
