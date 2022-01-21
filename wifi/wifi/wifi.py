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

from protocol.srv import Wifi

from protocol.srv import IP

import rclpy
import operator
import subprocess

g_node = None

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


def wifi_connect(request, response):
    global g_node

    response.result = return_connect_status(
        nmcliConnectWifi(request.ssid, request.pwd))
    return response


def get_ip(request, response):
    global g_node

    response.ip = getIP('wlan0')
    return response


def main(args=None):
    global g_node
    rclpy.init(args=args)

    g_node = rclpy.create_node('wifi')

    srv_wifi = g_node.create_service(Wifi, 'wifi', wifi_connect)
    srv_ip = g_node.create_service(IP, 'ip', get_ip)
    while rclpy.ok():
        rclpy.spin_once(g_node)

    # Destroy the service attached to the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    g_node.destroy_service(srv_wifi)
    g_node.destroy_service(srv_ip)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
