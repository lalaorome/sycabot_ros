import sys
import time
from xml.dom import INDEX_SIZE_ERR
import numpy as np
import math as m

from sycabot_control.CtrllerActionServer import CtrllerActionServer

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Pose2D
from sycabot_interfaces.action import Control

class PPCtrller(CtrllerActionServer):
    # https://blog.actorsfit.com/a?ID=01550-fd216bc6-e1d5-4d16-b901-143d2b19c430
    def __init__(self):
        super().__init__('PPCtrller')
        self.declare_parameter('LookAhead_dist', 0.1) # [m]

        self.LAD = self.get_parameter('LookAhead_dist').value
        self._action_server.action_name = 'test'

    def control_cb(self, goal_handle):
        result = Control.Result()

        path = goal_handle.request.path # Array of Pose2D
        interpolate = True
        path_x, path_y = self.get_path_x_y(path, interpolate)
        idx = 0
        self.wait4pose()
        t_init = time.time()

        while time.time() - t_init < 100. :
            idx = self.get_target(path_x, path_y)
            path_ref = Pose2D()
            path_ref.x = path_x[idx]
            path_ref.y = path_y[idx]
            self.viz_pathref_pub.publish(path_ref)
            v,w = self.generatePPinputs(path_x[idx], path_y[idx])
            Vr,Vl = self.velocities2wheelinput(v,w)
            self.sendVelCmd(Vr,Vl)
            time.sleep(self.Ts)
        self.stop()
        time.sleep(self.Ts)
        goal_handle.succeed()
        result.success = True
        return result
    
    def get_target(self,path_x, path_y):
        '''
        # Find the index of the closest point to the current position of the vehicle.

        arguments :
            path_x, path_y (list) = list of points describing the path to follow.
        ------------------------------------------------
        return :
            idx (int) : index of the closest point
        '''

        dx = [self.rob_state[self.X] - x for x in path_x]
        dy = [self.rob_state[self.Y] - y for y in path_y]

        d =  [abs(m.sqrt(id_x**2+id_y**2)) for  (id_x,id_y) in zip(dx,dy) ] 
        idx = d.index(min(d)) 
        L =  d[idx]
        Lf = 0.1*self.velocity[self.LINEAR] + self.LAD

        # Search look Ahead target Point index 
        while L<Lf and idx != len(path_x)-1:
            # if idx == len(path_x)-1 : idx = len(path_x)-1
            L = d[idx]
            idx +=  1

        return idx
    
    def get_path_x_y(self, path, interpolate):
        path_x = [pose.x for pose in path]
        path_y = [pose.y for pose in path]
        if interpolate : path_x, path_y = self.interpolate_path(path_x,path_y)
        return path_x, path_y

    def interpolate_path(self,path_x,path_y, step_size = 0.01):
        new_path_x = []
        new_path_y = []
        endpoint = False
        
        for i in range(len(path_x)):
            if i == len(path_x)-1 : 
                endpoint = True
                dist = np.max([np.linalg.norm(path_x[i] - path_x[0]), np.linalg.norm(path_y[i] - path_y[0])])
                resolution = int(dist/step_size)
                new_path_x = np.concatenate((new_path_x,np.linspace(path_x[i], path_x[0], resolution, endpoint=endpoint)))
                new_path_y = np.concatenate((new_path_y,np.linspace(path_y[i], path_y[0], resolution, endpoint=endpoint)))
            else :
                dist = np.max([np.linalg.norm(path_x[i] - path_x[i+1]), np.linalg.norm(path_y[i] - path_y[i+1])])
                resolution = int(dist/step_size)
                new_path_x = np.concatenate((new_path_x,np.linspace(path_x[i], path_x[i+1], resolution, endpoint=endpoint)))
                new_path_y = np.concatenate((new_path_y,np.linspace(path_y[i], path_y[i+1], resolution, endpoint=endpoint)))
        return new_path_x, new_path_y
    
    def generatePPinputs(self, target_x, target_y):

        alpha =self.rob_state[self.THETA] - m.atan2(self.rob_state[self.Y]-target_y,self.rob_state[self.X]-target_x)

        R = self.LAD/(2*m.sin(alpha))
        w = 2*self.max_ang_vel*m.sin(alpha)/self.LAD
        v = self.max_lin_vel
        return v,w



def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = PPCtrller()
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