import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException

from sycabot_interfaces.msg import Pose2D
from sycabot_interfaces.action import Control

import numpy as np
import random

class DeadzoneActionClient(Node):

    def __init__(self):
        super().__init__('control_action_client')

        self.declare_parameter('SycaBot_id', 1)

        self.Sycabot_id = self.get_parameter('SycaBot_id').value
        
        self._action_client = ActionClient(self, Control, f'/SycaBot_W{self.Sycabot_id}/PPCtrller_start_control')

    def send_goal(self, order):
        goal_msg = Control.Goal()
        self._action_client.wait_for_server()

        poses = self.create_tajectory_randpoints()
        goal_msg.path = poses
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.success))
        rclpy.shutdown()
    
    def create_tajectory_randpoints(self):
        poses = []
        points = [[0.,0.],[-1.352, -0.840], [-0.088,1.409],[1.306,-0.948],[0.869,2.150],[-1.155,2.208],[-0.067,-1.547],[0.,-0.4],[0.3,0.],[0.,0.]]
        for p in points :
            pose = Pose2D()
            pose.x = p[0]
            pose.y = p[1]
            poses.append(pose)
        return poses

    def create_square_trajectory(self):
        poses = []
        pose = Pose2D()
        pose.x = 1.
        pose.y = 1.
        poses.append(pose)
        pose = Pose2D()
        pose.x = -1.
        pose.y = 1.
        poses.append(pose)
        pose = Pose2D()
        pose.x = -1.
        pose.y = -1.
        poses.append(pose)
        pose = Pose2D()
        pose.x = 1.
        pose.y = -1.
        poses.append(pose)
        pose = Pose2D()
        pose.x = 1.
        pose.y = 1.
        poses.append(pose)
        return poses
    
    def create_circle_trajectory(self,r ,n):
        poses = []
        time = np.linspace(0, 2*np.pi, n, endpoint=False)
        for t in time :
            pose = Pose2D()
            pose.x = r * np.cos(t)
            pose.y = r * np.sin(t)
            poses.append(pose)
        return poses

def main(args=None):
    rclpy.init(args=args)

    action_client = DeadzoneActionClient()

    action_client.send_goal(10)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()