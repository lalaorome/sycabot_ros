from telnetlib import GA
import numpy as np
from .Gridworld import Gridworld
from sycabot_utils.utilities import quat2eul
import time
import math as m
import sycabot_central.GAMEPAD as GAMEPAD
from evdev import InputDevice


import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from sycabot_interfaces.action import Empty, Control
from sycabot_interfaces.msg import Motor, Pose2D


class GamepadActionServer(Node):

    RIGHT_WHEEL = 0
    LEFT_WHEEL = 1
    X=0
    Y=1
    THETA=2
    Ts = 0.1
    LINEAR = 0
    ANGULAR = 1

    def __init__(self):
        super().__init__('Gamepad_controller')

        self.declare_parameter('id', 1)
        self.declare_parameter('max_velocity', 0.3)
        self.declare_parameter('grid_size', 0.1) #step size for the grid 
        
        self.id = self.get_parameter('id').value
        self.max_vel = self.get_parameter('max_velocity').value
        self.grid_size = self.get_parameter('grid_size').value

        self.rob_state = np.array([False,False,False]) # x,y,theta: [-pi,pi]
        self.current_centroid_idx = None
        self.start = False
        self.gridworld  = Gridworld(self.grid_size)
        self.cmd_dict = {
            'up': 0,
            'down': 1,
            'left': 2,
            'right': 3,
        }
        
        cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            Empty,
            f'/SycaBot_W{self.id}/gamepad_control',
            self.gamepad_cb, callback_group=cb_group)
        
        self. _action_client = ActionClient(self,Empty,f'/SycaBot_W{self.id}/gamepad_control')
        self. _MPC_action_client = ActionClient(self, Control, f'/SycaBot_W{self.id}/MPC_start_control')

        
        # Subscribe to pose topic
        self.pose_sub = self.create_subscription(PoseStamped, f'/mocap_node/SycaBot_W{self.id}/pose', self.get_pose_cb, 10, callback_group=cb_group)
        # Create motor publisher
        self.vel_cmd_pub = self.create_publisher(Motor, f'/SycaBot_W{self.id}/cmd_vel', 10, callback_group=cb_group)

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
            self.start_action()
            self.start = True
        return
    
    def start_action(self):
        self.wait4pose()
        goal_msg = Empty.Goal()
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

    def gamepad_cb(self, goal_handle):
        #creates object 'gamepad' to store the data
        gamepad = InputDevice('/dev/input/event3')
        self.wait4pose()
        centroid_idx=self.gridworld.get_centroid(self.rob_state)
        goal = None

        #evdev takes care of polling the controller in a loop
        for event in gamepad.read_loop():
            if event.type != 0 :
                if event.code == GAMEPAD.ARROW_UP_DOWN:
                    if event.value == GAMEPAD.ARROW_UP_PRESSED :
                        goal = self.gridworld.get_next_goal(centroid_idx,'up')
                        self.send_goal(goal)
                    elif event.value == GAMEPAD.ARROW_DOWN_PRESSED :
                        goal = self.gridworld.get_next_goal(centroid_idx,'down')      
                        self.send_goal(goal)            
                elif event.code == GAMEPAD.ARROW_RIGHT_LEFT :
                    if event.value == GAMEPAD.ARROW_RIGHT_PRESSED :
                        goal = self.gridworld.get_next_goal(centroid_idx,'right')
                        self.send_goal(goal)
                    elif event.value == GAMEPAD.ARROW_LEFT_PRESSED :
                        goal = self.gridworld.get_next_goal(centroid_idx,'left')
                        self.send_goal(goal)
                else : goal = goal

    def wait4pose(self):
        # Initialisation : Wait for pose
        while not np.all(self.rob_state) :
                time.sleep(0.1)
                self.get_logger().info('No pose yet, waiting again...\n')
        return

    def send_goal(self, goal):
        '''
        Send a goal request to the MPC Action Client.
        Compute a path of wayposes and their synchronized times. 

        arguments :
        ------------------------------------------------
        return :
        '''
        print('here')
        goal_msg = Control.Goal()
        self._action_client.wait_for_server()
        wayposes = []
        wayposes_times = []
        wayposes, wayposes_times = self.add_syncronised_waypose(wayposes, wayposes_times, 0., goal, 1.)

        path = []
        for i in range(len(wayposes_times)):
            pose = Pose2D()
            pose.x = wayposes[0,i]
            pose.y = wayposes[1,i]
            pose.theta = wayposes[2,i]
            path.append(pose)
        print(path)
        goal_msg.path = path
        goal_msg.timestamps = wayposes_times.tolist()
        self._send_goal_future = self._MPC_action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
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
        
        return new_poses, new_times
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

    def generate_point_from_command(self, cmd):
        '''
        Create a path of pose2D from the points defined on the floor.

        arguments :
        ------------------------------------------------
        return :
            poses (list) : List of pose2D points.
        '''
        poses = []
        points = [[0.,0.],[-1.352, -0.840], [-0.088,1.409],[1.306,-0.948],[0.869,2.150],[-1.155,2.208],[-0.067,-1.547],[0.,-0.4],[0.3,0.],[0.,0.]]
        for p in points :
            pose = Pose2D()
            pose.x = p[0]
            pose.y = p[1]
            poses.append(pose)
        return poses

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = GamepadActionServer()
    while not np.all(node.rob_state) :
        time.sleep(0.1)
        rclpy.spin_once(node,timeout_sec=0.1)
   
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

