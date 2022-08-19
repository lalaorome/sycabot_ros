from telnetlib import GA
import numpy as np
from .Gridworld import Gridworld
from .Gamepad_recorder import recorder
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
        self.declare_parameter('grid_size', 0.5) #step size for the grid 
        
        self.id = self.get_parameter('id').value
        self.max_vel = self.get_parameter('max_velocity').value
        self.grid_size = self.get_parameter('grid_size').value

        self.rob_state = np.array([False,False,False]) # x,y,theta: [-pi,pi]
        self.centroid_idx = None
        self.start = False
        self.gridworld  = Gridworld(self.grid_size)
        self.recorder = recorder()
        self.cmd_dict = {
            'up': 0,
            'down': 1,
            'left': 2,
            'right': 3,
        }
        
        cb_group = ReentrantCallbackGroup()

        # Create action server for polling the gamepad inputs and generating commands base on them. 
        self._action_server = ActionServer(
            self,
            Empty,
            f'/SycaBot_W{self.id}/gamepad_control',
            self.gamepad_cb, callback_group=cb_group)
        
        # Create the action clients for the gamepad action and the MPC
        self. _gamepad_action_client = ActionClient(self,Empty,f'/SycaBot_W{self.id}/gamepad_control')
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
        self._gamepad_action_client.wait_for_server()
        self._send_goal_future = self._gamepad_action_client.send_goal_async(goal_msg)

    def gamepad_cb(self, goal_handle):
        '''
        This is the GamepadActionServer callback, it is in charge of continously polling the Gamepad inputs and it executes the 
        appropriate commands based on that. 
        '''
        #creates object 'gamepad' to store the data
        gamepad = InputDevice('/dev/input/event3')
        self.wait4pose()
        self.centroid_idx=self.gridworld.get_centroid(self.rob_state)
        goal = None

        #evdev takes care of polling the controller in a loop
        for event in gamepad.read_loop():
            if event.type != 0 :
                if event.code == GAMEPAD.ARROW_UP_DOWN:
                    if event.value == GAMEPAD.ARROW_UP_PRESSED :
                        input = 'up'
                        goal = self.gridworld.get_next_goal(self.centroid_idx,input)
                        self.send_goal(goal)
                        if self.recorder.recording : self.recorder.record(input, self.centroid_idx)
                    elif event.value == GAMEPAD.ARROW_DOWN_PRESSED :
                        input = 'down'
                        goal = self.gridworld.get_next_goal(self.centroid_idx,input)      
                        self.send_goal(goal)
                        if self.recorder.recording : self.recorder.record(input, self.centroid_idx)
                elif event.code == GAMEPAD.ARROW_RIGHT_LEFT :
                    if event.value == GAMEPAD.ARROW_RIGHT_PRESSED :
                        input = 'right'
                        goal = self.gridworld.get_next_goal(self.centroid_idx,input)
                        self.send_goal(goal)
                        if self.recorder.recording : self.recorder.record(input, self.centroid_idx)
                    elif event.value == GAMEPAD.ARROW_LEFT_PRESSED :
                        input = 'left'
                        goal = self.gridworld.get_next_goal(self.centroid_idx,input)
                        self.send_goal(goal)
                        if self.recorder.recording : self.recorder.record(input, self.centroid_idx)
                elif event.code == GAMEPAD.BUTTON_X :
                    if event.value == GAMEPAD.BUTTON_DOWN :
                        self.recorder.reinit_recording()
                elif event.code == GAMEPAD.BUTTON_B :
                    if event.value == GAMEPAD.BUTTON_DOWN :
                        self.recorder.stop_resume_recording()
                elif event.code == GAMEPAD.BUTTON_A :
                    if event.value == GAMEPAD.BUTTON_DOWN :
                        self.recorder.new_run()
                elif event.code == GAMEPAD.BUTTON_START :
                    if event.value == GAMEPAD.BUTTON_DOWN :
                        self.recorder.start_recording()
                elif event.code == GAMEPAD.BUTTON_SELECT :
                    if event.value == GAMEPAD.BUTTON_DOWN :
                        self.recorder.save_recording()

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
        goal_msg = Control.Goal()
        self._MPC_action_client.wait_for_server()
        wayposes = np.transpose([self.rob_state])
        wayposes_times = np.array([0.])
        # wayposes, wayposes_times = self.add_syncronised_waypose(wayposes, wayposes_times, 0., self.rob_state[:2], 0.)
        wayposes, wayposes_times = self.add_syncronised_waypose(wayposes, wayposes_times, 0., goal, 1.5)

        path = []
        for i in range(len(wayposes_times)):
            pose = Pose2D()
            pose.x = wayposes[0,i]
            pose.y = wayposes[1,i]
            pose.theta = wayposes[2,i]
            path.append(pose)
        goal_msg.path = path
        goal_msg.timestamps = wayposes_times.tolist()
        self._send_goal_future = self._MPC_action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def add_syncronised_waypose(self, current_poses, current_waypose_times, current_t,next_waypoint,next_travel_duration):
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

            new_poses = np.zeros((3,W+2))
            new_times = np.zeros(W+2)
            new_poses[:,:W] = reduced_poses
            new_times[:W] = reduced_times
            new_poses[0,W] = reduced_poses[0,-1]
            new_poses[1,W] = reduced_poses[1,-1]
            new_poses[2,W] = np.arctan2(next_waypoint[1] - reduced_poses[1,-1], next_waypoint[0] - reduced_poses[0,-1])
            new_times[W] = reduced_times[-1] + 0.5
            new_poses[0,W + 1] = next_waypoint[0]
            new_poses[1,W + 1] = next_waypoint[1]
            new_poses[2,W + 1] = new_poses[2,W]
            new_times[W + 1] = new_times[W] + next_travel_duration
        else : 
            new_poses = np.zeros((3,1))
            new_poses[:2,0] = next_waypoint[:2]
            new_poses[2] = 0.
            new_times = np.array([current_t])
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
        self.centroid_idx=self.gridworld.get_centroid(self.rob_state)
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

    def print_instructions(self):
        line = "\n##############################################################################################################\n"
        sentence = "Possible commands are : \n"
        record = "\t\tstart : Start a recording.\n"
        stop = "\t\tB : Resume or pause the current recording. It doesn't save it. It doesn't reinitialise it.\n"
        new = "\t\tA : Start a new run to record.\n"
        reinit="\t\tX : Reinitialise the current run recording. In case of mistake or bad behavior.\n"
        save = "\t\tselect : Save the recording."
        print(line + sentence + record + stop + new + reinit + save + line)

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = GamepadActionServer()
    while not np.all(node.rob_state) :
        time.sleep(0.1)
        rclpy.spin_once(node,timeout_sec=0.1)
    node.print_instructions()
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

