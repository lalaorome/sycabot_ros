import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from message_filters import ApproximateTimeSynchronizer, Subscriber

from sycabot_utils.utilities import quat2eul

import numpy as np
from numpy.linalg import norm
from scipy.optimize import linear_sum_assignment, minimize
import math as m
import time

from std_srvs.srv import Trigger
from sycabot_interfaces.srv import BeaconSrv, Task
from sycabot_interfaces.msg import Pose2D
from geometry_msgs.msg import PoseStamped

class cCAPT(Node):
    MAX_LIN_VEL = 0.3
    def __init__(self):
        super().__init__("cCAPT_pathplanner")

        self.initialised = False
        self.jb_positions = None
        self.OptiTrack_sub = []
        self.goals = None
        cb_group = ReentrantCallbackGroup()
        # Define get ids service client
        self.get_ids_cli = self.create_client(BeaconSrv, 'get_list_ids')
        while not self.get_ids_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Get ids service not available, waiting again...\n')
        
        # Define get ids service client
        self.refresh_ids_cli = self.create_client(Trigger, 'refresh_list_ids')
        while not self.refresh_ids_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Refresh ids service not available, waiting again...\n')
        
        # Create service for task request
        self.task_srv = self.create_service(Task, 'cCAPT_task_srv', self.set_task_cb, callback_group=cb_group)

        self.get_ids()
        self.initialise_pose_acquisition()
        self.initialise_goals()
        

    def initialise_goals(self):
        '''
        Generate the goals using cCAPT. 

        arguments :
        ------------------------------------------------
        return :
        '''
        self.wait4pose()
        N = len(self.ids)
        D=np.zeros((N,N))
        robot_radius = 0.13
        D_min = 2 * np.sqrt(2) * robot_radius
        while self.goals is None :
            goals = np.random.rand(len(self.ids),2)
            goals[:,0] = goals[:,0]*4 - 2.
            goals[:,1] = goals[:,1]*6 - 3.
            for i in range(N):
                for j in range(N):
                    D[i,j] = norm(goals[i] - goals[j])
                    if i==j : D[i,j] = 900.
            if np.all(D>0.3) : 
                self.goals = goals
        self.cCAPT(vmax = self.MAX_LIN_VEL, t0=0.)

    def set_task_cb(self, request, response):
        '''
        Callback for the tasks service. Sends the goal.

        arguments :
            request (interfaces.srv/Start.Response) =
                id (int64) = identifier of the jetbot [1,2,3,...]
        ------------------------------------------------
        return :
            response (interfaces.srv/Task.Response) = 
                task (geometry_msgs.msg/Pose) = pose of the assigned task
        '''
        idx = np.where(self.ids==request.id)[0][0]
        
        wayposes, wayposes_times = np.transpose([self.jb_positions[idx]]), np.array([0.])
        wayposes, wayposes_times = self.add_syncronised_waypose(wayposes, wayposes_times, 0., self.goals[idx], self.tf)
        tasks = []
        for i in range(len(wayposes_times)):
            pose = Pose2D()
            pose.x = wayposes[0,i]
            pose.y = wayposes[1,i]
            pose.theta = wayposes[2,i]
            tasks.append(pose)

        response.tasks = tasks
        response.tfs = wayposes_times.tolist()
        return response

    def get_ids(self):
        '''
        Get the IDs from the ebacon node. Call and refresh until the beacon give a success.

        arguments :
        ------------------------------------------------
        return :
        '''
        get_ids_req = BeaconSrv.Request()
        self.future = self.get_ids_cli.call_async(get_ids_req)
        rclpy.spin_until_future_complete(self, self.future)
        while not self.future.result().success :
            self.future = self.refresh_ids()
            get_ids_req = BeaconSrv.Request()
            self.future = self.get_ids_cli.call_async(get_ids_req)
            rclpy.spin_until_future_complete(self, self.future)
        self.ids = np.array(self.future.result().ids)
        return

    def refresh_ids(self):
        '''
        Refresh the IDs of the beacon node.

        arguments :
        ------------------------------------------------
        return :
        '''
        refresh_ids_req = Trigger.Request()
        self.future = self.refresh_ids_cli.call_async(refresh_ids_req)
        rclpy.spin_until_future_complete(self, self.future)
        return

    def cCAPT(self, vmax=1., t0=0):
        '''
        Task assignment algorithm. c-CAPT algo adapted from the work of M. Turpin : https://journals.sagepub.com/doi/full/10.1177/0278364913515307
        Algorithm developped for the case where M = N and cost function is difference of velocities squared.

        arguments :
            vmax (float) = max speed of the robot [m/s]
            t0   (float) = initial starting time [s]
        ------------------------------------------------
        return :
        '''
        # Step 1 : initialise varibales N,M, D and phi
        N,M = self.jb_positions.shape[0], self.goals.shape[0]
        D=np.zeros((N,M))
        phi=np.zeros((N,M))
        # Step 2 : Compute cost matrix for each position and goal
        for i in range(N):
            for j in range(M):
                D[i,j] = norm(self.jb_positions[i,0:2] - self.goals[j])**2
        # Step 3 : Compute phi_star using a simple optimizer
        row_idx, col_idx = linear_sum_assignment(D)
        phi[row_idx, col_idx] = 1
        tf = max(norm(self.jb_positions[:,0:2] - self.goals[col_idx], axis=1))/vmax    

        # Step 4 : Compute trajectory and the task where goals[i] corresponds to jebtot[i]
        self.goals = phi@self.goals
        self.tf = tf
        return

    def add_syncronised_waypose(self, current_poses: list, current_waypose_times: list, 
                                current_t: float, next_waypoint: list, next_travel_duration: float):
        '''
        Add a new waypose to the current list of poses, synchronizes it in time, and makes the robot turn when it is arrived.
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
    
    def initialise_pose_acquisition(self):
        '''
        Initialise the poses acuquistion by synchronizing all the topics and using the same callback.  

        arguments :
        ------------------------------------------------
        return :
        '''
        # Create sync callback group to get all the poses
        cb_group = ReentrantCallbackGroup()
        for id in self.ids:
            self.OptiTrack_sub.append(Subscriber(self, PoseStamped, f"/mocap_node/SycaBot_W{id}/pose"))
        self.ts = ApproximateTimeSynchronizer(self.OptiTrack_sub, queue_size=10, slop = 1.)
        self.ts.registerCallback(self.get_jb_pose_cb)

    def get_jb_pose_cb(self, *poses):
        '''
        Get and gather jetbot positions.
        arguments :
            *poses (PoseStamped) = array containing the position of the jetbots
        ------------------------------------------------
        return :
        '''
        quat = [poses[0].pose.orientation.x, poses[0].pose.orientation.y, poses[0].pose.orientation.z, poses[0].pose.orientation.w]
        theta = quat2eul(quat)
        self.jb_positions = np.array([[poses[0].pose.position.x, poses[0].pose.position.y, theta]])
        for p in poses[1:] :
            quat = [p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w]
            theta = quat2eul(quat)
            self.jb_positions = np.append(self.jb_positions, np.array([[p.pose.position.x, p.pose.position.y, theta]]), axis=0)
    
        return

    def wait4pose(self):
        while self.jb_positions is None :
            time.sleep(0.1)
            self.get_logger().info(f"Waiting for positions ...")
            rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = cCAPT()
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
