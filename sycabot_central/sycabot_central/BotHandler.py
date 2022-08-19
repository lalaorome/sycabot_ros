from array import array
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import ReentrantCallbackGroup


from sycabot_interfaces.srv import Task
from sycabot_interfaces.action import Control 
from sycabot_interfaces.msg import Pose2D
from geometry_msgs.msg import PoseStamped


import numpy as np
import time
import math as m
from sycabot_utils.utilities import quat2eul


class BotHandler(Node):
    def __init__(self, sycabot_id, controller : str, pathplanner: str ):
        super().__init__(f"SycaBot_W{sycabot_id}_handler")
        self.id = sycabot_id
        self.rob_state = np.array([False,False,False])
        self.wayposes, self.wayposes_times = [],[]
        qos = qos_profile_sensor_data
        cb_group = ReentrantCallbackGroup()
        # Define action client for MPC control
        self._action_client = ActionClient(self, Control, f'/SycaBot_W{self.id}/{controller}_start_control')
        # Define get task service client
        self.get_task_cli = self.create_client(Task, f'{pathplanner}_task_srv')
        while not self.get_task_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Get task service not available, waiting again...\n')
       
    def ask_task(self):
        '''
        Ask the task for its Sycabot to the path planner service.
        '''
        task_req = Task.Request()
        task_req.id = self.id
        self.future = self.get_task_cli.call_async(task_req)
    
    def set_task(self):
        '''
        Get the task for its sycabot once the result from the path planner service arrived.
        '''
        self.tasks = self.future.result().tasks
        self.tfs = self.future.result().tfs

    def send_goal(self):
        '''
        Synchronise the path with time and send a goal request to the MPCActionServer.
        '''
        try :
            # Wait for Action server
            self._action_client.wait_for_server()
            # Generate the path and format4 it to correspond to the Goal message formate (list of Pose2D)
            goal_msg = Control.Goal()
        except Exception as e :
            print(e)
        # Send the goal request and had a done callback to know wethr the goal was accepted or not.
        goal_msg.path = self.tasks
        goal_msg.timestamps = self.tfs
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        '''
        Get the response from the MPCActionServer to know wether the goal was accepted or not. 
        '''
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        # Add a done callback to execute when the action finishes.
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        '''
        Get the result from the Action Server. (Success : True or False)
        '''
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.success))