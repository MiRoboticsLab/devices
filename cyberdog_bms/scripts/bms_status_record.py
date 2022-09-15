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

from pickle import TRUE
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import String
from protocol.msg import BmsStatus
from sensor_msgs.msg import Range
import os
import sys
import time
import threading
import logging

logger = logging.getLogger(__name__)
logger.setLevel(level=logging.DEBUG)

# StreamHandler
stream_handler = logging.StreamHandler(sys.stdout)
stream_handler.setLevel(level=logging.DEBUG)
logger.addHandler(stream_handler)

# FileHandler
file_handler = logging.FileHandler('bms_status.log')
file_handler.setLevel(level=logging.INFO)
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('sensors_record')
        self.get_logger().info("======begin sensors_record======")
        '''
        os.system("gnome-terminal -e 'ros2 topic hz /bms_status'")
        '''

        logger.info('电量%	电压mV 电流mA 电池温度（度） 无线充温度（度） 循环次数n 健康度n 状态')
        
        self.batt_volt = 0
        self.batt_soc = 0
        self.batt_temp = 0
        self.batt_loop_number = 0 
        self.batt_health = 0
        self.status = 0

        self.bms_sub = self.create_subscription(BmsStatus, 'bms_status', self.bms_callback ,10)

    def bms_callback(self, msg):
        # uint16 batt_volt 
        # int16 batt_curr
        # uint8 batt_soc
        # int16 batt_temp
        # uint8 batt_st
        # uint8 key_val
        # uint8 disable_charge
        # uint8 power_supply
        # uint8 buzze
        # uint8 status
        # int8  batt_health
        # int16 batt_loop_number
        # int8  powerboard_status
        # 电量%	 电压mV	电流mA	电池温度（度） 适配器温度（度） 无线充温度（度） 循环次数n	健康度n	电池状态	状态

        self.batt_volt = msg.batt_volt
        self.batt_soc = msg.batt_soc
        self.batt_temp = msg.batt_temp
        self.batt_loop_number = msg.batt_loop_number 
        self.batt_health = msg.batt_health
        self.status = msg.batt_st

        record_log()

    def record_log(self):
        logger.info('%f, %f, %f, %f, %f, %d, %d, %X',
        self.batt_soc,
        self.batt_volt,
        self.batt_curr,
        self.batt_temp,
        0.0,
        self.batt_loop_number,
        self.batt_health,
        self.status)

    def test_log(self):
        logger.info('%02f, %02f, %02f, %02f, %02f, %04d, %04d, %02X',
        100,
        100,
        100,
        34,
        0.0,
        100,
        100,
        7)
        
def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    for num in range(1, 20):
        minimal_subscriber.test_log()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()