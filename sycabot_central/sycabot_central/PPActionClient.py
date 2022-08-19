import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sycabot_interfaces.msg import Pose2D
from sycabot_interfaces.action import Control
from geometry_msgs.msg import PoseStamped
from sycabot_utils.utilities import quat2eul

import numpy as np
import time
import math as m

class PPActionClient(Node):

    def __init__(self):
        super().__init__('control_action_client')

        self.declare_parameter('SycaBot_id', 1)
        
        self.Sycabot_id = self.get_parameter('SycaBot_id').value

        self.add_on_set_parameters_callback(self.parameters_callback)
        
        self.rob_state = np.array([False,False,False]) # x,y,theta: [-pi,pi]
        self.velocity = np.array([0.,0.])
        self.previous_state  = np.array([0.,0.,0.])
        self.start = False

    def intialise(self):
        cb_group = ReentrantCallbackGroup()
        self._action_client = ActionClient(self, Control, f'/SycaBot_W{self.Sycabot_id}/PPCtrller_start_control', callback_group=cb_group)
        self.pose_sub = self.create_subscription(PoseStamped, f'/mocap_node/SycaBot_W{self.Sycabot_id}/pose', self.get_pose_cb, 10, callback_group=cb_group)
    
    def destroy_links(self):
        self._action_client.destroy()
        self.pose_sub.destroy()

    def parameters_callback(self, params):
        for param in params :
            if param.name == 'id' :
                self.Sycabot_id = self.param.value
        self.destroy_links()
        self.intialise()


    def get_pose_cb(self, p):
        '''
        Get jetbot positions.

        arguments :
            p (PoseStamped) = position of the jetbots
        ------------------------------------------------
        return :
        '''
        quat = [p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w]
        theta = quat2eul(quat)
        self.previous_state = self.rob_state
        self.rob_state = np.array([p.pose.position.x, p.pose.position.y, theta])
        self.time = float(p.header.stamp.sec) + float(p.header.stamp.nanosec)*10e-10
        if not self.start :
            self.send_goal()
            self.start = True
        return

    def send_goal(self):
        '''
        Send a goal request to the PPCtrller Action Client.
        Compute a path of wayposes and their synchronized times. 

        arguments :
        ------------------------------------------------
        return :
        '''
        goal_msg = Control.Goal()
        self._action_client.wait_for_server()

        init_pose = Pose2D()
        init_pose.x = self.rob_state[0]
        init_pose.y = self.rob_state[1]
        init_pose.theta = self.rob_state[2]
        path = self.create_tajectory_randpoints()
        path.insert(0,init_pose)

        goal_msg.path = path
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

    def wait4pose(self):
        '''
        Wait for the pose to be published.

        arguments :
        ------------------------------------------------
        return :
        '''
        # Initialisation : Wait for pose
        while not np.all(self.rob_state) :
                self.get_logger().info('No pose yet, waiting again...\n')
                rclpy.spin_once(self, timeout_sec=0.01)

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = PPActionClient()
    node.intialise()
    node.wait4pose()
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