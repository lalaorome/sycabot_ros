from logging import raiseExceptions
import sys
import time
import numpy as np
from sycabot_utils.utilities import utilities as ut

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import ExternalShutdownException
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from sycabot_interfaces.action import Deadzone
from sycabot_interfaces.msg import Motor


class DeadzoneActionServer(Node):
    def __init__(self):
        super().__init__('deadzone_action_server')

        self.declare_parameter('id', 1)
        self.declare_parameter('deadzones', [0., 0., -0., -0.])

        self.id = self.get_parameter('id').value
        self.deadzones = self.get_parameter('deadzones').value
        cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            Deadzone,
            f'/SycaBot_W{self.id}/deadzones_identification',
            self.compute_deadzones_cb, callback_group=cb_group)

        self.rob_state = np.array([999.,0.,0.]) # x,y,theta: [-pi,pi]
        self.RIGHT_WHEEL = 0
        self.LEFT_WHEEL = 1
        
        # Subscribe to pose topic
        self.pose_sub = self.create_subscription(PoseStamped, f'/mocap_node/SycaBot_W{self.id}/pose', self.get_pose_cb, 1, callback_group=cb_group)

        # Create motor publisher
        self.vel_cmd_pub = self.create_publisher(Motor, f'/SycaBot_W{self.id}/cmd_vel', 10, callback_group=cb_group)
        

    def compute_deadzones_cb(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = Deadzone.Result()
        try :
            self.deadzone()
            indent = '  '
            with open('config/params_identification.yaml', 'r') as file:
                # read a list of lines into data
                data = file.readlines()
                data[3] = f"{indent}{indent}deadzones: [{self.deadzones[0]:.3f},{self.deadzones[1]:.3f},{self.deadzones[2]:.3f},{self.deadzones[3]:.3f}]\n" 
                with open('config/params_identification.yaml', 'w') as file:
                    file.writelines( data )
        except Exception as e:
            self.get_logger().error("Couldn't perform deadzone identification.\n Here is the error message :\n" + str(e))
            goal_handle.abort()
            return result
        goal_handle.succeed()
        result.deadzones = self.deadzones
        
        return result

    def get_pose_cb(self, p):
        '''
        Get jetbot positions.

        arguments :
            p (PoseStamped) = position of the jetbots
        ------------------------------------------------
        return :
        '''
        quat = [p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w]
        theta = ut.quat2eul(quat)
        self.rob_state = np.array([p.pose.position.x, p.pose.position.y, theta])
        self.time = float(p.header.stamp.sec) + float(p.header.stamp.nanosec)*10e-10
        return
    
    def deadzone(self, n_runs=1):
        cmd_vel = Motor()

        for j in range(4): # Right and left Wheel deadzone identification
            for _ in range(n_runs):
                deadzone = []
                states = [self.rob_state]
                
                k = 0
                moved = False
                while not moved :
                    states.append(self.rob_state)
                    if abs(states[-1][2] - states[-2][2]) > 0.2 : # If robot has moved
                        if j%2 == self.RIGHT_WHEEL : deadzone.append(cmd_vel.right)
                        else : deadzone.append(cmd_vel.left)
                        cmd_vel.left, cmd_vel.right = 0.,0.
                        self.vel_cmd_pub.publish(cmd_vel) # Stop the robot
                        moved = True
                        time.sleep(1.)
                    else :
                        if j == self.RIGHT_WHEEL : cmd_vel.right = 0.01 + k*0.01
                        elif j == self.LEFT_WHEEL : cmd_vel.left = 0.01 + k*0.01
                        elif j%2 == self.RIGHT_WHEEL : cmd_vel.right = -0.01 - k*0.01
                        else : cmd_vel.left = -0.01 - k*0.01
                        k+=1
                    self.vel_cmd_pub.publish(cmd_vel)
                    time.sleep(1.)

            self.deadzones[j] = np.max(deadzone)

        cmd_vel.left, cmd_vel.right = 0.,0.
        self.vel_cmd_pub.publish(cmd_vel) # Stop the robot
        return self.deadzones


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = DeadzoneActionServer()

    # Initialisation : Wait for pose
    while node.rob_state[0] == 999. :
            time.sleep(0.1)
            node.get_logger().info('No pose yet, waiting again...\n')
            rclpy.spin_once(node)
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
    