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

        self.declare_parameter('SycaBot_id', 1)
        
        self.Sycabot_id = self.get_parameter('SycaBot_id').value

        self.add_on_set_parameters_callback(self.parameters_callback)
        
        self.rob_state = np.array([False,False,False]) # x,y,theta: [-pi,pi]
        self.velocity = np.array([0.,0.])
        self.previous_state  = np.array([0.,0.,0.])
        self.start = False

    def intialise(self):
        '''
        Initialise the MPC Action client and the pose subscriber.

        arguments :
        ------------------------------------------------
        return :
        '''
        cb_group = ReentrantCallbackGroup()
        self._action_client = ActionClient(self, Control, f'/SycaBot_W{self.Sycabot_id}/MPC_start_control', callback_group=cb_group)
        self.pose_sub = self.create_subscription(PoseStamped, f'/mocap_node/SycaBot_W{self.Sycabot_id}/pose', self.get_pose_cb, 1, callback_group=cb_group)
    
    def destroy_links(self):
        '''
        Destroy the MPC Action client and the pose subscriber.

        arguments :
        ------------------------------------------------
        return :
        '''
        self._action_client.destroy()
        self.pose_sub.destroy()

    def parameters_callback(self, params):
        '''
        Parameter callback. Called when parameters are modified.

        arguments :
        ------------------------------------------------
        return :
        '''
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
        '''
        Send a goal request to the MPC Action Client.
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
        path, times = self.create_tajectory_frompoints()
        times.insert(0,0.)
        path.insert(0,init_pose)

        goal_msg.path = path
        goal_msg.timestamps = times
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        '''
        Called when there is a response from the MPC Action server. 
        Do nothing for now.

        arguments :
        ------------------------------------------------
        return :
        '''
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        '''
        Called when there is a result from the MPC Action server. 
        Do nothing for now.

        arguments :
        ------------------------------------------------
        return :
        '''
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.success))
        if result.success :
            response = input("Do you want to start again ? [y/n]")
            while response != 'y' and response != 'n' :
                print('Wrong input please press "y" or "n" and press Enter')
                response = input("Do you want to start again ? [y/n]")
            if response == 'y' :
                print("Let's go for another ride !")
                self.send_goal()
            else :
                print('Exiting...')

    def create_tajectory_frompoints(self):
        '''
        Create a path of pose2D from the points defined on the floor.

        arguments :
        ------------------------------------------------
        return :
            poses (list) : List of pose2D points.
        '''
        poses = []
        times = []
        points = [[0.,0.],[-1.352, -0.840], [-0.088,1.409],[1.306,-0.948],[0.869,2.150],[-1.155,2.208],[-0.067,-1.547],[0.,-0.4],[0.3,0.],[0.,0.]]
        for p, i in zip(points, range(len(points))) :
            pose = Pose2D()
            pose.x = p[0]
            pose.y = p[1]
            poses.append(pose)
            times.append(5.+i*5.)
        return poses, times

    def add_syncronised_waypose(self, current_poses, current_waypose_times, current_t,next_waypoint,next_travel_duration):
        '''
        Add the time to the wayposes.
        Remove the past wayposes.

        arguments :
            current_poses (np.array) [3,N] : Current list of wayposes
            current_wayposes_times (list) [N] : Current list of synchronized times
            current_t (float64) : Current time
            next_waypoint (np.array) [x,y] : Next waypoint to add to the wayposes
            next_travel_duration (float64) : Completion time to go to the next waypoint
        ------------------------------------------------
        return :
            new_poses (np.array) [3,N+1] : Updated list of wayposes with the next waypoint
            new_times (list) [N+1] : updated list of synchronized times with the next travel duration
        '''
        
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
        '''
        Wait for the pose to be published.

        arguments :
        ------------------------------------------------
        return :
        '''
        # Initialisation : Wait for pose
        while not np.all(self.rob_state) :
                self.get_logger().info('No pose yet, waiting again...\n')
                rclpy.spin_once(self, timeout_sec = 0.1)

        return


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = MPCActionClient()
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