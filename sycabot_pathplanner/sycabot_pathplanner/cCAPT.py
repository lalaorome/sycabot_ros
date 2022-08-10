from asyncio import futures
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from message_filters import ApproximateTimeSynchronizer, Subscriber

from sycabot_utils.utilities import quat2eul

import numpy as np
from numpy.linalg import norm
from scipy.optimize import linear_sum_assignment, minimize
import math as m
import time
import sys

from std_srvs.srv import Trigger
from sycabot_interfaces.srv import BeaconSrv, Task
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point

class cCAPT(Node):
    MAX_LIN_VEL = 0.3
    def __init__(self):
        super().__init__("cCAPT_pathplanner")

        self.initialised = False
        self.jb_positions = None
        self.OptiTrack_sub = []
        self.ids = None
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
        self.task_srv = self.create_service(Task, 'task_srv', self.set_task_cb, callback_group = cb_group)

    async def set_task_cb(self, request, response):
        '''
        Compute goals if it has never been init and give it to the asking jetbot.

        arguments :
            request (interfaces.srv/Start.Response) =
                id (int64) = identifier of the jetbot [1,2,3,...]
                position (Pose) = pose of the jetbot
        ------------------------------------------------
        return :
            response (interfaces.srv/Task.Response) = 
                task (geometry_msgs.msg/Pose) = pose of the assigned task
        '''
        if not self.initialised :
            await self.get_ids()
            self.initialise_pose_acquisition()
            while self.jb_positions is None :
                time.sleep(0.1)
                self.get_logger().info("Waiting for positions...")

            good_goals = False
            N = len(self.ids)
            D=np.zeros((N,N))
            
            while self.goals is None :
                goals = np.random.rand(len(self.ids),2)
                goals[:,0] = goals[:,0]*4 - 2.
                goals[:,1] = goals[:,1]*6 - 3.

                for i in range(N):
                    for j in range(N):
                        D[i,j] = norm(goals[i] - goals[j])
                        if i==j : D[i,j] = 900.

                if np.all(D>0.4) : 
                    self.goals = goals

            self.cCAPT(vmax = self.MAX_LIN_VEL, t0=0.)

        # Step 2 : Compute and send response if id is the good one
        task = Point()
        
        task.x = self.goals[request.id-1,0]
        task.y = self.goals[request.id-1,1]
        task.z = 0.

        response.task = task
        response.tf = self.tf

        return response
        

    async def get_ids(self):
        get_ids_req = BeaconSrv.Request()
        future = self.get_ids_cli.call_async(get_ids_req)
        try :
            get_ids = await future
        except Exception as e:
            self.get_logger().info('Get ids service call failed %r' % (e,))

        if not get_ids.success :
            self.get_logger().info(f'{get_ids.message}')
            refresh_req = Trigger.Request()
            future = self.refresh_ids_cli.call_async(refresh_req)
            try :
                result = await future
            except Exception as e:
                self.get_logger().info('Refresh ids service call failed %r' % (e,))
            else :
                get_ids_req = BeaconSrv.Request()
                future = self.get_ids_cli.call_async(get_ids_req)
                try :
                    result = await future
                except Exception as e:
                    self.get_logger().info('Get ids service call failed %r' % (e,))
                else :
                    self.ids = result.ids
        else : 
            self.ids = np.array(get_ids.ids)
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
        print(self.jb_positions, self.goals)
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
    
    def initialise_pose_acquisition(self):
        # Create sync callback group to get all the poses
        for id in self.ids:
            self.OptiTrack_sub.append(Subscriber(self, PoseStamped, f"/mocap_node/SycaBot_W{id}/pose"))
        self.ts = ApproximateTimeSynchronizer(self.OptiTrack_sub, queue_size=10, slop = 0.1)
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
