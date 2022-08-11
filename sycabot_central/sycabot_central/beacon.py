from argparse import Namespace
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from sycabot_interfaces.msg import BeaconMsg
from sycabot_interfaces.srv import BeaconSrv
from std_srvs.srv import Trigger
from rclpy.executors import MultiThreadedExecutor

import numpy as np
import math as m
import time
import sys

class beacon(Node):
    '''
    A beacon is responsible for doing the link between the robots and any user by collecting the ids of the SycaBots that are publishing on the beacon topic.
    It is possible to refresh the list if it is not updated.
    '''
    def __init__(self):
        super().__init__('beacon', namespace='central')
        self.ids = []
        self.prev_count = 0
        cb_group = ReentrantCallbackGroup()
        self.beacon_sub = self.create_subscription(BeaconMsg, 'beacon',  self.get_jetbot_ids_cb, 10, callback_group = cb_group)
        self.send_ids_srv = self.create_service(BeaconSrv, 'get_list_ids', self.send_ids_cb)
        self.refresh_ids = self.create_service(Trigger, 'refresh_list_ids', self.refresh_cb)
        self.check_updated = self.create_timer(0.1, self.check_updated_cb, callback_group = cb_group)
    
    def get_jetbot_ids_cb(self, msg):
        if msg.id not in self.ids :
            self.ids.append(msg.id)
            self.ids.sort()        
        self.get_logger().info('ids : ' + str(self.ids))
    
    def refresh_cb(self, request, response):
        self.ids = []
        response.success = True
        return response
    
    def send_ids_cb(self, request, response):
        if self.updated :
            response.success = True
            response.ids = self.ids
            print(self.ids)
        else :
            response.success = False
            response.message = 'Not yet udpated. Try to refresh.'
        return response

    def check_updated_cb(self):
        if len(self.ids) == self.count_publishers('/beacon') : self.updated = True
        else : self.updated = False



def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = beacon()
    executor.add_node(node)
    try :
        executor.spin()
    except Exception as e :
        print(e)
    finally:
        executor.shutdown()
        node.destroy_node()
    return

if __name__ == '__main__':
    main()
