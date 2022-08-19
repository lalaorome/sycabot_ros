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


class BotHandlerMPC(Node):
    def __init__(self, sycabot_id):
        super().__init__(f"SycaBot_W{sycabot_id}_handler")
        self.id = sycabot_id
        self.rob_state = np.array([False,False,False])
        self.wayposes, self.wayposes_times = [],[]
        qos = qos_profile_sensor_data
        cb_group = ReentrantCallbackGroup()
        # Define action client for MPC control
        self._action_client = ActionClient(self, Control, f'/SycaBot_W{self.id}/MPC_start_control')
        # Define get task service client
        self.get_task_cli = self.create_client(Task, 'task_srv')
        while not self.get_task_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Get task service not available, waiting again...\n')
        # Create pose subscriber
        self.pose_sub = self.create_subscription(PoseStamped, f'/mocap_node/SycaBot_W{self.id}/pose', self.get_pose_cb,qos, callback_group = cb_group)
    
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
        return

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
        self.waypoint = self.future.result().task
        self.tf = self.future.result().tf
    
    def init_wayposes(self):
        '''
        Initialise the path with its sycabot initial pose.
        '''
        self.wayposes, self.wayposes_times = np.transpose([self.rob_state]), np.array([0.])

    def send_goal(self):
        '''
        Synchronise the path with time and send a goal request to the MPCActionServer.
        '''
        try :
            # Wait for Action server
            self._action_client.wait_for_server()
            # Generate the path and format4 it to correspond to the Goal message formate (list of Pose2D)
            goal_msg = Control.Goal()
            self.wayposes, self.wayposes_times = self.add_syncronised_waypose(self.wayposes, self.wayposes_times, 0., np.array([self.waypoint.x,self.waypoint.y]), self.tf)
            path = []
            for i in range(len(self.wayposes_times)):
                pose = Pose2D()
                pose.x = self.wayposes[0,i]
                pose.y = self.wayposes[1,i]
                pose.theta = self.wayposes[2,i]
                path.append(pose)
        except Exception as e :
            print(e)
        # Send the goal request and had a done callback to know wethr the goal was accepted or not.
        goal_msg.path = path
        goal_msg.timestamps = self.wayposes_times.tolist()
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

    def add_syncronised_waypose(self, current_poses: array, current_waypose_times: array, 
                                current_t: float, next_waypoint: array, next_travel_duration: float):
        '''
        Add a new waypose to the current list of poses, synchronizes it in time, and makes the robot trun when it is arrived.
        arguments :
            current_poses [3,T] = position of the jetbots
            current_waypose_time [T] = times corresponding to each waypose
            current_t = current time of the execution
            next_waypoint: next point to add to the path
            next_travel_duration : Time to go to travel to the next waypoint
        ------------------------------------------------
        return :
            new_poses [3,T] : Updated path with the next waypoint
            new_times [T] : Updated times with the arrival time synchronized with the new waypoint 
        '''
        if np.any(current_poses):
            idx_poses_after_t = np.argwhere(current_waypose_times > current_t)
            if idx_poses_after_t.size > 0:
                idx_next = idx_poses_after_t[0]
                if idx_next > 1: #if there are more than one waypoint in list that have been passed
                    reduced_poses = current_poses[:,idx_next - 1:]
                    reduced_times = current_waypose_times[idx_next - 1:]
                else:
                    reduced_poses = current_poses
                    reduced_times = current_waypose_times
            else:
                reduced_poses = current_poses
                reduced_times = current_waypose_times

            W = len(reduced_poses[0,:])    

            rounds = 3

            new_poses = np.zeros((3,W + 2 + rounds * 4))
            new_times = np.zeros(W + 2 + rounds * 4)
            new_poses[:,:W] = reduced_poses
            new_times[:W] = reduced_times
            new_poses[0,W] = reduced_poses[0,-1]
            new_poses[1,W] = reduced_poses[1,-1]
            new_poses[2,W] = np.arctan2(next_waypoint[1] - reduced_poses[1,-1], next_waypoint[0] - reduced_poses[0,-1])
            new_times[W] = reduced_times[-1] + 1
            new_poses[0,W + 1] = next_waypoint[0]
            new_poses[1,W + 1] = next_waypoint[1]
            new_poses[2,W + 1] = new_poses[2,W]
            new_times[W + 1] = new_times[W] + next_travel_duration
            

            dir = np.sign(np.random.randn(1))
            for ts in range(rounds * 4):
                new_poses[0,W + 2 + ts] = next_waypoint[0]
                new_poses[1,W + 2 + ts] = next_waypoint[1]
                new_poses[2,W + 2 + ts] = np.remainder(new_poses[2,W + 2 + ts - 1] + dir * m.pi / 2 + m.pi,2 * m.pi) - m.pi
                new_times[W + 2 + ts] = new_times[W + 2 + ts - 1] + 0.5
        else :
            new_poses = np.zeros((3,1))
            new_poses[:2,0] = next_waypoint[:2]
            new_poses[2] = 0.
            new_times = np.array([current_t])
        
        return new_poses, new_times

    def wait4pose(self):
        # Initialisation : Wait for pose
        while not np.all(self.rob_state) :
            time.sleep(0.1)
            self.get_logger().info('No pose yet, waiting again...\n')
            rclpy.spin_once(self, timeout_sec=0.1)

        return
