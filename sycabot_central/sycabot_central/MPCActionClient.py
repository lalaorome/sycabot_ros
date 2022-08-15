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

class MPCActionClient(Node):

    def __init__(self):
        super().__init__('control_action_client')

        self.declare_parameter('SycaBot_id', 2)
        
        self.Sycabot_id = self.get_parameter('SycaBot_id').value

        self.add_on_set_parameters_callback(self.parameters_callback)
        
        self.rob_state = np.array([False,False,False]) # x,y,theta: [-pi,pi]
        self.velocity = np.array([0.,0.])
        self.previous_state  = np.array([0.,0.,0.])
        self.start = False

    def intialise(self):
        cb_group = ReentrantCallbackGroup()
        self._action_client = ActionClient(self, Control, f'/SycaBot_W{self.Sycabot_id}/MPC_start_control', callback_group=cb_group)
        self.pose_sub = self.create_subscription(PoseStamped, f'/mocap_node/SycaBot_W{self.Sycabot_id}/pose', self.get_pose_cb, 1, callback_group=cb_group)
    
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
            print(self.start)
        return

    def send_goal(self):
        self.wait4pose()
        goal_msg = Control.Goal()
        self._action_client.wait_for_server()

        init_pose = Pose2D()
        init_pose.x = self.rob_state[0]
        init_pose.y = self.rob_state[1]
        init_pose.theta = self.rob_state[2]
        path = self.create_tajectory_randpoints()
        path.insert(0,init_pose)

        wayposes, wayposes_times = [],[]
        for p in path:
            wayposes, wayposes_times = self.add_syncronised_waypose(wayposes, wayposes_times, 0., np.array([p.x,p.y]), 10.)
        # print(wayposes, wayposes_times)

        path = []
        for i in range(len(wayposes_times)):
            pose = Pose2D()
            pose.x = wayposes[0,i]
            pose.y = wayposes[1,i]
            pose.theta = wayposes[2,i]
            path.append(pose)
        goal_msg.path = path
        goal_msg.timestamps = wayposes_times.tolist()
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

    def add_syncronised_waypose(self, current_poses, current_waypose_times, current_t,next_waypoint,next_travel_duration):
        
        new_poses = np.zeros((3,1))
        new_poses[:2,0] = next_waypoint[:2]
        new_poses[2] = 0.
        new_times = np.array([current_t])
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
        
        return new_poses, new_times
    def wait4pose(self):
        # Initialisation : Wait for pose
        while not np.all(self.rob_state) :
                time.sleep(0.1)
                self.get_logger().info('No pose yet, waiting again...\n')

        return
    def generate_reference_trajectory_from_timed_wayposes(self, current_state, wayposes, waypose_times,t,Ts,N,mode = 'go_straight_or_turn'):
        x_pos_ref = np.ones(N + 1)*current_state[0]
        y_pos_ref = np.ones(N  + 1)*current_state[1]
        theta_ref = np.ones(N  + 1)*current_state[2]
        v_ref = np.zeros(N + 1)
        omega_ref = np.zeros(N + 1)
        
        if mode == 'ignore_corners':
            t_vec = t + np.linspace(0,N * Ts, N + 1)
            for k in range(N + 1):
                idx_poses_after_t = np.argwhere(waypose_times > t_vec[k])
                if idx_poses_after_t.size > 0:
                    idx_k = idx_poses_after_t[0]
                    if idx_k > 0:
                        v_ref[k] = np.sqrt((wayposes[1,idx_k] - wayposes[1,idx_k - 1]) ** 2 + (wayposes[0,idx_k] - wayposes[0,idx_k - 1]) ** 2) / (waypose_times[idx_k] - waypose_times[idx_k - 1])
                        theta_ref[k] = np.arctan2(wayposes[1,idx_k] - wayposes[1,idx_k - 1], wayposes[0,idx_k] - wayposes[0,idx_k - 1])
                        l = (t_vec[k] - waypose_times[idx_k - 1]) / (waypose_times[idx_k] - waypose_times[idx_k - 1])
                        x_pos_ref[k] = l * wayposes[0,idx_k] + (1 - l) * wayposes[0,idx_k - 1]
                        y_pos_ref[k] = l * wayposes[1,idx_k] + (1 - l) * wayposes[1,idx_k - 1]
        
        if mode == 'stop_in_corners':
            t_vec = t + np.linspace(0,N * Ts, N + 1)
            for k in range(N + 1):
                idx_poses_after_t = np.argwhere(waypose_times > t_vec[k])
                if idx_poses_after_t.size > 0:
                    idx_k = idx_poses_after_t[0]
                    if idx_k > 0:
                        v_ref[k] = np.sqrt((wayposes[1,idx_k] - wayposes[1,idx_k - 1]) ** 2 + (wayposes[0,idx_k] - wayposes[0,idx_k - 1]) ** 2) / (waypose_times[idx_k] - waypose_times[idx_k - 1])
                        if np.remainder(idx_k,2) == 0:
                            l = (t_vec[k] - waypose_times[idx_k - 1]) / (waypose_times[idx_k] - waypose_times[idx_k - 1])
                            theta_ref[k] = np.arctan2(wayposes[1,idx_k] - wayposes[1,idx_k - 1], wayposes[0,idx_k] - wayposes[0,idx_k - 1])
                            x_pos_ref[k] = l * wayposes[0,idx_k] + (1 - l) * wayposes[0,idx_k - 1]
                            y_pos_ref[k] = l * wayposes[1,idx_k] + (1 - l) * wayposes[1,idx_k - 1]
                        else:
                            x_pos_ref[k] = wayposes[0,idx_k - 1]
                            y_pos_ref[k] = wayposes[1,idx_k - 1]
                            l_rot = (t_vec[k] - waypose_times[idx_k - 1]) / (waypose_times[idx_k] - waypose_times[idx_k - 1])
                            # print(l_rot)
                            theta_ref[k]  = wayposes[2,idx_k - 1] + l_rot *  np.arctan2(np.sin(wayposes[2,idx_k] - wayposes[2,idx_k - 1]),np.cos(wayposes[2,idx_k] - wayposes[2,idx_k - 1]))
                            omega_ref[k] = np.arctan2(np.sin(wayposes[2,idx_k] - wayposes[2,idx_k - 1]),np.cos(wayposes[2,idx_k] - wayposes[2,idx_k - 1])) / (waypose_times[idx_k] - waypose_times[idx_k - 1])
                    else:
                        v_ref[k] = np.sqrt((wayposes[1,idx_k] - wayposes[1,idx_k - 1]) ** 2 + (wayposes[0,idx_k] - wayposes[0,idx_k - 1]) ** 2) / (waypose_times[idx_k] - waypose_times[idx_k - 1])
        
        if mode == 'go_straight_or_turn':
            t_vec = t + np.linspace(0,N * Ts, N + 1)
            for k in range(N + 1):
                idx_poses_after_t = np.argwhere(waypose_times > t_vec[k])
                if idx_poses_after_t.size > 0:
                    idx_k = idx_poses_after_t[0]
                    if idx_k > 0:
                        v_ref[k] = np.sqrt((wayposes[1,idx_k] - wayposes[1,idx_k - 1]) ** 2 + (wayposes[0,idx_k] - wayposes[0,idx_k - 1]) ** 2) / (waypose_times[idx_k] - waypose_times[idx_k - 1])
                        if v_ref[k] != 0:
                            l = (t_vec[k] - waypose_times[idx_k - 1]) / (waypose_times[idx_k] - waypose_times[idx_k - 1])
                            theta_ref[k] = np.arctan2(wayposes[1,idx_k] - wayposes[1,idx_k - 1], wayposes[0,idx_k] - wayposes[0,idx_k - 1])
                            x_pos_ref[k] = l * wayposes[0,idx_k] + (1 - l) * wayposes[0,idx_k - 1]
                            y_pos_ref[k] = l * wayposes[1,idx_k] + (1 - l) * wayposes[1,idx_k - 1]
                        else:
                            x_pos_ref[k] = wayposes[0,idx_k - 1]
                            y_pos_ref[k] = wayposes[1,idx_k - 1]
                            l_rot = (t_vec[k] - waypose_times[idx_k - 1]) / (waypose_times[idx_k] - waypose_times[idx_k - 1])
                            # print(l_rot)
                            theta_ref[k]  = wayposes[2,idx_k - 1] + l_rot *  np.arctan2(np.sin(wayposes[2,idx_k] - wayposes[2,idx_k - 1]),np.cos(wayposes[2,idx_k] - wayposes[2,idx_k - 1]))
                            omega_ref[k] = np.arctan2(np.sin(wayposes[2,idx_k] - wayposes[2,idx_k - 1]),np.cos(wayposes[2,idx_k] - wayposes[2,idx_k - 1])) / (waypose_times[idx_k] - waypose_times[idx_k - 1])
                    else:
                        v_ref[k] = np.sqrt((wayposes[1,idx_k] - wayposes[1,idx_k - 1]) ** 2 + (wayposes[0,idx_k] - wayposes[0,idx_k - 1]) ** 2) / (waypose_times[idx_k] - waypose_times[idx_k - 1])


        state_ref = np.vstack((x_pos_ref.reshape(1,N + 1), y_pos_ref.reshape(1,N + 1), theta_ref.reshape(1,N + 1)))
        input_ref = np.vstack((v_ref[:-1].reshape(1,N), omega_ref[:-1].reshape(1,N)))
        return state_ref, input_ref



def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = MPCActionClient()
    node.intialise()
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