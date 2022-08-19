from logging import raiseExceptions
import sys
import time
import numpy as np
from sycabot_utils.utilities import quat2eul, p2vel
import matplotlib.pyplot as plt
from sklearn import linear_model
import math as m

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import ExternalShutdownException
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from sycabot_interfaces.action import Identification
from sycabot_interfaces.msg import Motor

plot = True


class IdentificationActionServer(Node):
    def __init__(self):
        super().__init__('identification_action_server')

        self.declare_parameter('id', 1)
        self.declare_parameter('deadzones', [0.100,0.090,-0.110,-0.140])
        self.declare_parameter('f_coefs', [35.85760339557938, 36.162967632872])
        self.declare_parameter('sampling_period', 0.01)
        self.declare_parameter('wheel_radius', 0.060325/2)
        self.declare_parameter('L', 0.1016)

        self.id = self.get_parameter('id').value
        self.deadzones = self.get_parameter('deadzones').value
        self.f_coefs = self.get_parameter('f_coefs').value
        self.Ts = self.get_parameter('sampling_period').value
        self.R = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('L').value

        self.rob_state = np.array([False,False,False]) # x,y,theta: [-pi,pi]
        self.time_init = time.time()
        self.time = 0.
        self.RIGHT_WHEEL = 0
        self.LEFT_WHEEL = 1

        self.interpolate = True
        # information variables
        self.times = []
        self.states = []
        self.tspin = []
        self.tpub = []
        self.twait = []
        self.inputs = []
        self.time_stamps = []
        self.Ts_arr = []

        cb_group = ReentrantCallbackGroup()

        # Subscribe to pose topic
        self.pose_sub = self.create_subscription(PoseStamped, f'/mocap_node/SycaBot_W{self.id}/pose', self.get_pose_cb, 1, callback_group = cb_group)

        # Create motor publisher
        self.vel_cmd_pub = self.create_publisher(Motor, f'/SycaBot_W{self.id}/cmd_vel', 10, callback_group = cb_group)

        # Create action server
        self._action_server = ActionServer(
            self,
            Identification,
            f'/SycaBot_W{self.id}/start_identification',
            self.identification_cb, callback_group=cb_group)

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
        self.rob_state = np.array([p.pose.position.x, p.pose.position.y, theta])
        self.time = float(p.header.stamp.sec) + float(p.header.stamp.nanosec)*10e-10
        return
    
    def identification_cb(self, goal_handle):
        result = Identification.Result()
        n_inputs = goal_handle.request.n_inputs
        input_types = goal_handle.request.types
        
        cmd_vel = Motor()
        model = []
        u1, u2 = [], []

        # information variables
        self.times = []
        self.states = []
        self.tspin = []
        self.tpub = []
        self.twait = []
        self.inputs = []
        self.time_stamps = []
        self.Ts_arr = []

        self.wait4pose()

        t_init = time.time()
        # Generate inputs for the identification
        for t in input_types :
            n = int(n_inputs/len(input_types))
            tmp_u1, tmp_u2 = self.generate_inputs(self.Ts, self.deadzones, n=n, input=t, Vr_max=0.3, Vl_max=0.3)
            u1 = np.concatenate((u1, tmp_u1))
            u2 = np.concatenate((u2, tmp_u2))

        time.sleep(0.5)
        for i in range(n_inputs):
            tic = time.time()
            self.states.append(self.rob_state)
            self.time_stamps.append(self.time)
            ts = time.time()
            self.tspin.append(ts-tic)
            # To plot 
            self.times.append(time.time() - t_init)

            # Send velocity command to the robot
            cmd_vel.right, cmd_vel.left = u1[i],u2[i]
            self.vel_cmd_pub.publish(cmd_vel)
            self.inputs.append([cmd_vel.right, cmd_vel.left])
            model.append([(self.R)*cmd_vel.right/2, (self.R)*cmd_vel.left/2]) # [R*Ur/2, R*Ul/2] 
            model.append([(self.R)*cmd_vel.right/self.L, -(self.R)*cmd_vel.left/self.L]) # [R*Ur/L, -R*Ul/L]
            # to plot publish time
            tp = time.time()
            self.tpub.append(tp - ts)
            
            # Wait
            time.sleep(self.Ts)
            tw = time.time()
            self.twait.append(tw- tp)

            while(self.time_stamps[-1]==self.time):
                time.sleep(0.01)

        # Stop the robot
        cmd_vel.left, cmd_vel.right = 0.,0.
        self.vel_cmd_pub.publish(cmd_vel)

        # Transform lists to numpy arrays
        self.states = np.array(self.states)
        model = np.array(model)
        self.inputs = np.array(self.inputs)
        if self.interpolate :

            # Get velocity from measured inputs
            self.Ts_arr = [self.time_stamps[i+1]-self.time_stamps[i] for i in range(len(self.time_stamps)-1)]
            vel_m = p2vel(self.states,self.Ts_arr,n_inputs, self.inputs)

            f = linear_model.LinearRegression(fit_intercept=False)
            f.fit(model[:-2],vel_m)

            indent = '  '
            with open('config/params_identification.yaml', 'r') as file:
                # read a list of lines into data
                data = file.readlines()
            try :
                data[4] = f"{indent}{indent}f_coefs: [{f.coef_[0]}, {f.coef_[1]}]\n" 
                with open('config/params_identification.yaml', 'w') as file:
                    file.writelines( data )
            except Exception as e:
                self.get_logger().error("Couldn't perform identification.\n Here is the error message :\n" + str(e))
                goal_handle.abort()
                result.success = False
                return result
        
        else :
            # Get velocity from measured inputs
            self.Ts_arr = [self.time_stamps[i+1]-self.time_stamps[i] for i in range(len(self.time_stamps)-1)]
            velocities = np.zeros((n-1,2))
            for i in range(0,n-1) :
                idx = i
                velocities[i,0] = np.sign(self.inputs[idx,0]+self.inputs[idx,1])*m.sqrt(((self.states[idx+1,0]-self.states[idx,0])**2 + (self.states[idx+1,1]-self.states[idx,1])**2))/self.Ts_arr[idx] # v
                velocities[i,1] = (m.atan2(m.sin(self.states[idx+1,2]-self.states[idx,2]),m.cos(self.states[idx+1,2]-self.states[idx,2])))/self.Ts_arr[idx]
            self.vel_m = velocities

            self.plot_prediction()

        goal_handle.succeed()
        result.success = True
        return result

    def plot_prediction(self):
        
        v_predict = (self.f_coefs[0]*self.inputs[:,0]+ self.f_coefs[1]*self.inputs[:,1])*self.R/2
        w_predict = (self.f_coefs[0]*self.inputs[:,0]- self.f_coefs[1]*self.inputs[:,1])*self.R/self.L
        theta = [self.states[0,2]]
        x_predict = [self.states[0,0]]
        y_predict = [self.states[0,1]]
        for i in range(0,len(w_predict)):
            theta.append(theta[i] + w_predict[i]*self.Ts)
            x_predict.append(x_predict[i] + v_predict[i]*self.Ts*m.cos(theta[i]))
            y_predict.append(y_predict[i] + (v_predict[i]*self.Ts*m.sin(theta[i])))

        # Plot informations (Loop time, ....)
        fig,ax = plt.subplots(1,2,figsize=(16,8))

        ax[0].scatter(self.states[:,0],self.states[:,1], label="position measured")
        ax[0].scatter(x_predict, y_predict, label='predicted position')
        ax[0].set_ylabel('y [m]')
        ax[0].set_xlabel('x [m]')
        ax[0].legend(['measured', 'predicted'])
        ax[0].set_xlim(-2,2)
        ax[0].set_ylim(-4,4)

        ax[1].plot(self.times, self.states[:,2], '-', self.times, theta[1:], '--')
        ax[1].plot(self.times, [m.pi for i in range(len(self.times))], label='pi')
        ax[1].plot(self.times, [-m.pi for i in range(len(self.times))], label='-pi')
        ax[1].set_ylabel('angle [rad]')
        ax[1].set_xlabel('time [s]')
        ax[1].legend(['measured', 'predicted'])

        fig.suptitle('Sycabot_W'+str(self.id))

        # Plot communication and loop informations
        fig,ax = plt.subplots(1,2,figsize=(16,8))
        ax[0].bar('Loop Time', np.mean(self.tspin), width=0.35, yerr=np.std(self.tspin), label='mean spin time [s]')
        ax[0].bar('Loop Time', np.mean(self.tpub), width=0.35, yerr=np.std(self.tspin), bottom=np.mean(self.tspin), label='mean publish time [s]')
        ax[0].bar('Loop Time', np.mean(self.twait), width=0.35, yerr=np.std(self.tspin), bottom=np.mean(self.tspin) + np.mean(self.tpub), label='mean wait time [s]')
        ax[0].legend()

        ax[1].plot(self.times[:-1], self.Ts_arr)
        ax[1].set_ylabel('point by point frequency')
        ax[1].set_xlabel('time [s]')

        # Plot measured velocities
        fig,ax = plt.subplots(1,2,figsize=(16,8))

        ax[0].plot(self.times[:-1], self.vel_m[:,0], '-', self.times, v_predict, '--')
        ax[0].set_ylabel('linear velocity')
        ax[0].set_xlabel('time [s]')
        ax[0].legend(['measured', 'interpolate'])

        ax[1].plot(self.times[:-1], self.vel_m[:,1], '-', self.times, w_predict, '--')
        ax[1].set_ylabel('angular velocity')
        ax[1].set_xlabel('time [s]')
        ax[1].legend(['measured', 'predicted'])

        plt.legend()
        plt.show()

        
    def generate_inputs(self, Ts, deadzones, n=1,input='ramp', Vr_max=0.2, Vl_max=0.2, var_f = False):
        '''
        Generate velocity inputs for identification.

        arguments :
            Ts         (float)  = Sampling time
            Deadzones  (array) [R,L,-R,-L] = right and left wheels deadzones 
            n          (int)    = number of inputs to generate
            Vr_max, Vl_max     (float)  = Maximum input velocity to reach for left and right wheel
            input (string) = Type of input generation we want : 'ramp', 'step'
        ------------------------------------------------
        return :
            Vl,Vr (lists of inputs) [n] = wheel velocities
            v,w   (lists of inputs) [n] = linear (x_r) and angular (z_r) velocities (robot coordinates)
        '''
        right = []
        left = []

        #################################
        ##   Generate right,left inputs here  ##
        #################################

        
        if input=='ramp':
            for i in range(n) :
                right.append((Vr_max*(i+1)/n)+np.sign(Vr_max)*(deadzones[self.RIGHT_WHEEL]+0.03))
                left.append((Vl_max*(i+1)/n)+np.sign(Vl_max)*(deadzones[self.LEFT_WHEEL]+0.03))
                
        elif input=='step' :
            for i in range(n) :
                right.append(Vr_max)
                left.append(Vl_max)

        elif input=='random_sine':
            time = np.linspace(0,n*Ts, num=n)
            f1 = np.random.random()*0.3
            f2 = np.random.random()*0.3
            phase_shift = np.random.random()*2*np.pi
            right = np.sin(2 * np.pi * f1* time + phase_shift)*Vr_max/2 + Vr_max/2
            left = np.sin(2 * np.pi * f2 * time)*Vl_max/2 + Vl_max/2

        elif input=='random_sine_cosine':
            time = np.linspace(0,n*Ts, num=n)
            f = np.random.random()*0.6

            right = np.sin(2 * np.pi * f * time)*Vr_max*1.5
            left = np.cos(2 * np.pi * f * time)*Vl_max*1.5

        else :
            self.get_logger().warn('Wrong types message for identification action... Cannot generate inputs.')


        #################################

        return right,left

    def wait4pose(self):
        # Initialisation : Wait for pose
        while not np.all(self.rob_state) :
                time.sleep(0.1)
                self.get_logger().info('No pose yet, waiting again...\n')

        return

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = IdentificationActionServer()
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