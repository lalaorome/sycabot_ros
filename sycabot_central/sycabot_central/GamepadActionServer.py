import numpy as np
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

from geometry_msgs.msg import PoseStamped, Twist
from sycabot_interfaces.action import Empty
from sycabot_interfaces.msg import Motor


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
        super().__init__('controller')

        self.declare_parameter('id', 1)
        self.declare_parameter('max_velocity', 0.3)
        self.declare_parameter('grid_size', 0.01) #step size for the grid 
        
        self.id = self.get_parameter('id').value
        self.max_vel = self.get_parameter('max_velocity').value
        self.grid_size = self.get_parameter('grid_size').value


        cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            Empty,
            f'/SycaBot_W{self.id}/gamepad_control',
            self.gamepad_cb, callback_group=cb_group)
        
        self. _action_client = ActionClient(self,Empty,f'/SycaBot_W{self.id}/gamepad_control')

        self.rob_state = np.array([False,False,False]) # x,y,theta: [-pi,pi]
        self.velocity = np.array([0.,0.])
        self.velocity_measured = np.array([0.,0.])
        self.previous_state  = np.array([0.,0.,0.])
        # Subscribe to pose topic
        self.pose_sub = self.create_subscription(PoseStamped, f'/mocap_node/SycaBot_W{self.id}/pose', self.get_pose_cb, 10, callback_group=cb_group)
        # Create motor publisher
        self.vel_cmd_pub = self.create_publisher(Motor, f'/SycaBot_W{self.id}/cmd_vel', 10, callback_group=cb_group)
        self.start = False
        self.init_gridworld()

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
        self.velocity_measured = self.get_velocity()
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
        print('here')
        gamepad = InputDevice('/dev/input/event3')
        print('here')
        self.wait4pose()

        #evdev takes care of polling the controller in a loop
        for event in gamepad.read_loop():
            if event.type != 0 :
                print(event.code, GAMEPAD.BUTTON_B)
                if event.code == GAMEPAD.JOYSTICK_LEFT_UP_DOWN:
                    self.velocity[0] = (1 - event.value/GAMEPAD.JOYSTICK_MID_UDOWN)*self.max_vel
                if event.code == GAMEPAD.JOYSTICK_RIGHT_RIGHT_LEFT:
                    self.velocity[1] = (1 - event.value/GAMEPAD.JOYSTICK_MID_UDOWN)*self.max_vel*10
                if event.code == GAMEPAD.BUTTON_A :
                    if event.value == GAMEPAD.BUTTON_DOWN :
                        print('Button A')
                        self.velocity = np.array([0.,0.])                        
                if event.code == GAMEPAD.BUTTON_X :
                    if event.value == GAMEPAD.BUTTON_DOWN :
                        print('Button X')
                if event.code == GAMEPAD.BUTTON_B :
                    if event.value == GAMEPAD.BUTTON_DOWN :
                        print('here')
                        goal_handle.succeed()
                        result = Empty.Result()
                        result.finished = True
                        self.stop()
                        return result
                if event.code == GAMEPAD.BUTTON_Y :
                    if event.value == GAMEPAD.BUTTON_DOWN :
                        print('Button Y')
                if event.code == GAMEPAD.BUTTON_R1 :
                    if event.value == GAMEPAD.BUTTON_DOWN :
                        print('Button R1')
                if event.code == GAMEPAD.BUTTON_R2 :
                    if event.value == GAMEPAD.BUTTON_DOWN :
                        print('Button R2')
                if event.code == GAMEPAD.BUTTON_L1 :
                    if event.value == GAMEPAD.BUTTON_DOWN :
                        print('Button L1')
                if event.code == GAMEPAD.BUTTON_L2 :
                    if event.value == GAMEPAD.BUTTON_DOWN :
                        print('Button L2')
            
            cmd_vel = Motor()
            Vr,Vl = self.velocities2wheelinput()
            cmd_vel.right = Vr
            cmd_vel.left = Vl
            print('publishing')
            self.vel_cmd_pub.publish(cmd_vel)

    def wait4pose(self):
        # Initialisation : Wait for pose
        while not np.all(self.rob_state) :
                time.sleep(0.1)
                self.get_logger().info('No pose yet, waiting again...\n')

        return

    def sendVelCmd(self,Vr,Vl):
        vel_cmd = Motor()
        vel_cmd.right = Vr
        vel_cmd.left = Vl
        self.vel_cmd_pub.publish(vel_cmd)
    
    def stop(self):
        vel_cmd = Motor()
        vel_cmd.right = 0.
        vel_cmd.left = 0.
        self.vel_cmd_pub.publish(vel_cmd)

    def velocities2wheelinput(self):
        Vr = (self.velocity[0] + self.L*self.velocity[1]/2)/(self.R*self.f_coefs[0])
        Vl = (self.velocity[0] - self.L*self.velocity[1]/2)/(self.R*self.f_coefs[1])
        return Vr,Vl

    def get_velocity(self):
        v = np.linalg.norm(self.rob_state[0:2]-self.previous_state[0:2])/self.Ts
        w = m.atan2(m.sin(self.rob_state[2]-self.previous_state[2]),m.cos(self.rob_state[2]-self.previous_state[2]))/self.Ts
        return np.array([v,w])

    def init_gridworld(self):
        x = np.linspace(-2,2, int(4./self.grid_size))
        y = np.linspace(-4,4, int(8./self.grid_size))
        xv, yv = np.meshgrid(x,y)
        meshgrid_idx = np.transpose(np.stack((xv,yv), axis=0))
        print(meshgrid_idx[0,-1,:])

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = GamepadActionServer()
   
    # Initialisation : Wait for pose
    while node.rob_state[0] == 999. :
            time.sleep(0.1)
            node.get_logger().info('No pose yet, waiting again...\n')
            rclpy.spin_once(node, timeout_sec = 0.1)
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

