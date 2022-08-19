import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient

from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import ReentrantCallbackGroup

from sycabot_utils.utilities import quat2eul
from sycabot_central.BotHandlerMPC import BotHandlerMPC
from sycabot_interfaces.srv import BeaconSrv, Task
from sycabot_interfaces.action import Control 
from sycabot_interfaces.msg import Pose2D
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped

import numpy as np
import time
import math as m


class central(Node):
    '''
    This class is the "brain" of the execution. 
    It creates, destroy and coordinates the robot handlers to make them execute at the same time. 
    '''
    def __init__(self):
        super().__init__("central")

        self.ids = None
        self.handlers = []

        # Define get ids service client
        self.get_ids_cli = self.create_client(BeaconSrv, 'get_list_ids')
        while not self.get_ids_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Get ids service not available, waiting again...\n')
        
        # Define refresh ids service client
        self.refresh_ids_cli = self.create_client(Trigger, 'refresh_list_ids')
        while not self.refresh_ids_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Refresh ids service not available, waiting again...\n')
    
    def get_ids(self):
        '''
        Request the Ids frome the Beacon node.
        '''
        get_ids_req = BeaconSrv.Request()
        future = self.get_ids_cli.call_async(get_ids_req)
        return future

    def refresh_ids(self):
        '''
        Request a refresh of the ids from the beacon node.
        '''
        refresh_ids_req = Trigger.Request()
        future = self.get_ids_cli.call_async(refresh_ids_req)
        return future
    
    def create_handlers(self):
        '''
        Create the robot handlers for all the Sycabot present. 
        '''
        for id in self.ids :
            self.handlers.append(BotHandlerMPC(id))
        return
    
    def handlers_get_tasks(self, executor):
        '''
        Make all the handlers get their tasks.
        '''
        for bot in self.handlers :
            bot.ask_task()
            executor.spin_until_future_complete(bot, bot.future)
            bot.set_task()
    
    def handlers_send_goals(self):
        '''
        Make all the handlers send a goal request.
        '''
        for bot in self.handlers :
            bot.send_goal()
    
    def spin_all_handlers(self, executor):
        '''
        Spin all the handlers until execution is finished.
        '''
        for bot in self.handlers :
            executor.add_node(bot)
        executor.spin()
    
    def init_handlers(self):
        '''
        Initialise all the handlers, make them wait for their initial pose 
        and then initialise the path with it.
        '''
        for bot in self.handlers :
            bot.wait4pose()
            bot.init_wayposes()


def main(args=None):
    rclpy.init(args=args)
    Central = central()
    future = Central.get_ids()
    rclpy.spin_until_future_complete(Central, future)
    while not future.result().success :
        future = Central.refresh_ids()
        rclpy.spin_until_future_complete(Central, future)
        future = Central.get_ids()
        rclpy.spin_until_future_complete(Central, future)
    executor = MultiThreadedExecutor()
    Central.ids = future.result().ids
    Central.create_handlers()
    Central.handlers_get_tasks(rclpy)
    Central.init_handlers()
    Central.handlers_send_goals()
    Central.spin_all_handlers(executor)



if __name__ == '__main__':
    main()