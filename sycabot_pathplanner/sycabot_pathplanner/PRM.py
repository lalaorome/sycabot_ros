from asyncio import futures
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from message_filters import ApproximateTimeSynchronizer, Subscriber

from sycabot_utils.utilities import quat2eul

import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
import PRM_Node

# import numpy as np
# from numpy.linalg import norm
# from scipy.optimize import linear_sum_assignment, minimize
# import math as m
# import time
# import sys

from std_srvs.srv import Trigger
from sycabot_interfaces.srv import BeaconSrv, Task
from geometry_msgs.msg import PoseStamped
from sycabot_interfaces.msg import Pose2D

class PRM(Node):
    MAX_LIN_VEL = 0.3
    def __init__(self):
        super().__init__("PRM_pathplanner")

        self.initialised = False
        self.jb_positions = None
        self.OptiTrack_sub = []
        self.destinations = None
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
        self.task_srv = self.create_service(Task, 'PRM_task_srv', self.set_task_cb, callback_group=cb_group)

        self.get_ids()
        self.initialise_pose_acquisition()
        
        self.map_x_min = -1.5
        self.map_x_max = 1.5
        self.map_y_min = -3.5
        self.map_y_max = 3.5
        self.robot_radius = 0.13       
        self.initialise_obstacles()

        self.initialise_destinations()

        self.N_samples = 1000
        self.N_KNN = 10
        self.MAX_EDGE_LEN = 2.0
        self.plan_paths()
        
    def initialise_obstacles(self):
        ox = []
        oy = []
        N_x = math.ceil((self.map_x_max - self.map_x_min) / self.robot_radius) + 1
        N_y = math.ceil((self.map_y_max - self.map_y_min) / self.robot_radius) + 1
        for i in range(N_x):
            ox.append(self.x_min + i * (self.map_x_max - self.map_x_min) / (N_x - 1))
            oy.append(self.map_y_min)
        for i in range(N_y):
            ox.append(self.map_x_max)
            oy.append(self.y_min + i * (self.map_y_max - self.map_y_min) / (N_y - 1))
        for i in range(N_x):
            ox.append(self.x_min + i * (self.map_x_max - self.map_x_min) / (N_x - 1))
            oy.append(self.map_y_max)
        for i in range(N_y):
            ox.append(self.map_x_min)
            oy.append(self.y_min + i * (self.map_y_max - self.map_y_min) / (N_y - 1))
        for i in range(math.floor(N_x * 2 / 3)):
            ox.append(self.map_x_min + i * (self.map_x_max - self.map_x_min) / (N_x - 1))
            oy.append(-2.0)
        for i in range(math.floor(N_x * 2 / 3)):
            ox.append(self.map_x_max - i * (self.map_x_max - self.map_x_min) / (N_x - 1))
            oy.append(0.0)
        for i in range(math.floor(N_x * 2 / 3)):
            ox.append(self.map_x_max - i * (self.map_x_max - self.map_x_min) / (N_x - 1))
            oy.append(2.0)

        self.obstacle_kd_tree = KDTree(np.vstack((ox, oy)).T)
        return

    def initialise_destinations(self):
        N = len(self.ids)
        destinations = np.zeros((N,2))
        for rob in range(N):
            found_destination = False
            while not found_destination:
                gx = np.random.rand() * 3.0 - 1.5
                gy = np.random.rand() * 7.0 - 3.5

                dist, index = self.obstacle_kd_tree.query([gx, gy])
                if dist >= self.robot_radius:
                    destinations[rob,0] = gx
                    destinations[rob,1] = gy
                    found_destinations = True
        self.destinations = destinations
        return 


    def set_task_cb(self, request, response):
        '''
        Callback for the tasks service. Sends the goal.

        arguments :
            request (interfaces.srv/Start.Response) =
                id (int64) = identifier of the jetbot [1,2,3,...]
        ------------------------------------------------
        return :
            response (interfaces.srv/Task.Response) = 
                task (geometry_msgs.msg/Pose2D[]) = pose of the assigned task
        '''
        idx = np.where(self.ids==request.id)[0][0]
        tasks = []
        tfs = [] # Problem here should send angles and times
        for p in range(self.path_length[idx]):
            task_p = Pose2D()
            task_p.x = self.x_path[idx,p]
            task_p.y = self.y_path[idx,p]
            task_p.z = 0.
            tasks.append(task_p)

        response.tasks = tasks
        response.tf = 0.
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
        self.future = self.get_ids_cli.call_async(refresh_ids_req)
        rclpy.spin_until_future_complete(self, self.future)
        return

    def plan_path(self):
        self.N_samples = 1000
        self.sample_points()
        self.generate_road_map()
        N = len(self.ids)
        self.x_path = np.zeros((N,self.N_samples))
        self.y_path = np.zeros((N,self.N_samples))
        self.path_length = np.zeros(N)
        for rob in range(N):
            start_x = self.jb_positions[rob,0]
            start_y = self.jb.positions[rob,1]
            goal_x = self.destinations[rob,0]
            goal_y = self.destinations[rob,1]
            rx, ry = self.dijkstra_planning(start_x, start_y, goal_x, goal_y)
            self.path_length[rob] = len(rx)
            self.x_path[rob,:self.path_length[rob]] = rx
            self.y_path[rob,:self.path_length[rob]] = ry
        return
    
    def sample_points(self):
        sample_x, sample_y = [], []

        while len(sample_x) <= self.N_samples:
            tx = (np.random.rand() * (self.map_x_max - self.map_x_min)) + self.map_x_min
            ty = (np.random.rand() * (self.map_y_max - self.map_y_min)) + self.map_y_min

            dist, index = self.obstacle_kd_tree.query([tx, ty])

            if dist >= self.robot_radius:
                sample_x.append(tx)
                sample_y.append(ty)
        
        N = len(self.ids)
        for rob in range(N):
            sample_x.append(self.jb_positions[rob,0])
            sample_y.append(self.jb_positions[rob,1])
            sample_x.append(self.destinations[rob,0])
            sample_y.append(self.destinations[rob,1])
            self.sample_x = sample_x
            self.sample_y = sample_y
        return

    def generate_road_map(self):
        road_map = []
        N_sample_plus = len(self.sample_x)
        sample_kd_tree = KDTree(np.vstack((self.sample_x, self.sample_y)).T)

        for (i, ix, iy) in zip(range(N_sample_plus), self.sample_x, self.sample_y):

            dists, indexes = sample_kd_tree.query([ix, iy], k=N_sample_plus)
            edge_id = []

            for ii in range(1, len(indexes)):
                nx = self.sample_x[indexes[ii]]
                ny = self.sample_y[indexes[ii]]

                if not self.is_collision(ix, iy, nx, ny):
                    edge_id.append(indexes[ii])

                if len(edge_id) >= self.N_KNN:
                    break

            road_map.append(edge_id)   
            self.road_map = road_map
        return

    def is_collision(self,sx, sy, gx, gy):
        x = sx
        y = sy
        dx = gx - sx
        dy = gy - sy
        yaw = math.atan2(gy - sy, gx - sx)
        d = math.hypot(dx, dy)

        if d >= self.MAX_EDGE_LEN:
            return True

        D = self.robot_radius
        n_step = round(d / D)

        for i in range(n_step):
            dist, _ = self.obstacle_kd_tree.query([x, y])
            if dist <= self.robot_radius:
                return True  # collision
            x += D * math.cos(yaw)
            y += D * math.sin(yaw)

        # goal point check
        dist, _ = self.obstacle_kd_tree.query([gx, gy])
        if dist <= self.robot_radius:
            return True  # collision

        return False  # OK


    def dijkstra_planning(self,sx, sy, gx, gy):
        start_node = PRM_Node(sx, sy, 0.0, -1)
        goal_node = PRM_Node(gx, gy, 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[len(self.road_map) - 2] = start_node

        path_found = True

        while True:
            if not open_set:
                print("Cannot find path")
                path_found = False
                break

            c_id = min(open_set, key=lambda o: open_set[o].cost)
            current = open_set[c_id]

            if c_id == (len(self.road_map) - 1):
                print("goal is found!")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]
            # Add it to the closed set
            closed_set[c_id] = current

            # expand search grid based on motion model
            for i in range(len(self.road_map[c_id])):
                n_id = self.road_map[c_id][i]
                dx = self.sample_x[n_id] - current.x
                dy = self.sample_y[n_id] - current.y
                d = math.hypot(dx, dy)
                node = Node(self.sample_x[n_id], self.sample_y[n_id],
                            current.cost + d, c_id)

                if n_id in closed_set:
                    continue
                # Otherwise if it is already in the open set
                if n_id in open_set:
                    if open_set[n_id].cost > node.cost:
                        open_set[n_id].cost = node.cost
                        open_set[n_id].parent_index = c_id
                else:
                    open_set[n_id] = node

        if path_found is False:
            return [], []

        # generate final course
        rx, ry = [goal_node.x], [goal_node.y]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(n.x)
            ry.append(n.y)
            parent_index = n.parent_index

        return rx, ry

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
 

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = PRM()
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
