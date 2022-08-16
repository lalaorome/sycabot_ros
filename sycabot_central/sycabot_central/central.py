import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient

from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import ReentrantCallbackGroup


from sycabot_interfaces.srv import BeaconSrv, Task
from sycabot_interfaces.action import Control 
from sycabot_interfaces.msg import Pose2D
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped


import numpy as np
import time
import math as m
from sycabot_utils.utilities import quat2eul

class central(Node):
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
        get_ids_req = BeaconSrv.Request()
        future = self.get_ids_cli.call_async(get_ids_req)
        return future

    def refresh_ids(self):
        refresh_ids_req = Trigger.Request()
        future = self.get_ids_cli.call_async(refresh_ids_req)
        return future
    
    def create_handlers(self):
        for id in self.ids :
            self.handlers.append(bot_handler(id))
        return
    
    def handlers_get_tasks(self):
        for bot in self.handlers :
            bot.ask_task()
    
    def handlers_send_goals(self):
        for bot in self.handlers :
            bot.send_goal()

    def handlers_spin_until_futur_handlers(self, executor):
        for bot in self.handlers :
            executor.spin_until_future_complete(bot, bot.future)
            bot.set_task()
    
    def spin_all_handlers(self, executor):
        for bot in self.handlers :
            executor.add_node(bot)
        executor.spin()
    
    def init_handlers(self):
        for bot in self.handlers :
            bot.init_wayposes()

            
            

class bot_handler(Node):
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
        self.pose_sub = self.create_subscription(PoseStamped, f'/mocap_node/SycaBot_W{self.id}/pose', self.get_pose_cb, qos, callback_group = cb_group)
    
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
        task_req = Task.Request()
        task_req.id = self.id
        self.future = self.get_task_cli.call_async(task_req)
    
    def set_task(self):
        self.waypoint = self.future.result().task
        self.tf = self.future.result().tf
    
    def init_wayposes(self):
        self.wayposes, self.wayposes_times = self.add_syncronised_waypose(self.wayposes, self.wayposes_times, 0., np.array([self.rob_state[0],self.rob_state[1]]), self.tf)

    def send_goal(self):
        print('here', self.id)
        try :
            goal_msg = Control.Goal()
            self._action_client.wait_for_server()
            self.wait4pose()
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
        print('here', self.id)
        goal_msg.path = path
        goal_msg.timestamps = self.wayposes_times.tolist()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        print('here', self.id)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        print('here', self.id)
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

    def generate_reference_trajectory_from_timed_wayposes(self, current_state, wayposes, waypose_times,t,Ts,N,mode = 'ignore_corners'):
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

    def wait4pose(self):
        # Initialisation : Wait for pose
        while not np.all(self.rob_state) :
            time.sleep(0.01)
            self.get_logger().info('No pose yet, waiting again...\n')

        return

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
    Central.handlers_get_tasks()
    Central.handlers_spin_until_futur_handlers(rclpy)
    Central.init_handlers()
    Central.handlers_send_goals()
    Central.spin_all_handlers(executor)



if __name__ == '__main__':
    main()