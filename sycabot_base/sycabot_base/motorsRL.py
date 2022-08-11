import rclpy
import math

from rclpy.node import Node
from sycabot_interfaces.msg import Motor
from rcl_interfaces.msg import SetParametersResult

from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data

class MotorController(Node):
    """
    Abstract motor controller base node for supporting different JetBots.
    Can be extended to support any diff drive by overriding set_speed(),
    or any node that subscribes to the /jetbot/cmd_vel Twist message.
    """
    def __init__(self):
        super().__init__('motors')
        qos = qos_profile_sensor_data
        
        self.declare_parameter('id', 1)
        self.declare_parameter('left_trim', 0.0)
        self.declare_parameter('right_trim', 0.0)
        self.declare_parameter('max_pwm', 255)
        self.declare_parameter('max_rpm', 200)              # https://www.adafruit.com/product/3777
        self.declare_parameter('wheel_separation', 0.1016)  # 4 inches
        self.declare_parameter('wheel_diameter', 0.060325)  # 2 3/8 inches
        
        self.id = self.get_parameter('id').value
        self.left_trim = self.get_parameter('left_trim').value
        self.right_trim = self.get_parameter('right_trim').value
        self.max_pwm = self.get_parameter('max_pwm').value
        self.max_rpm = self.get_parameter('max_rpm').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.sub = self.create_subscription(Motor, f'/SycaBot_W{self.id}/cmd_vel', self.twist_listener, qos)
         
        self.last_x = -999
        self.last_rot = -999
        
    def destroy_node(self):
        self.get_logger().info(f"shutting down, stopping robot...")
        self.stop()
        
    def parameters_callback(self, params):
        for param in params:
            if param.name == 'left_trim':
                self.left_trim = param.value
            elif param.name == 'right_trim':
                self.right_trim = param.value
            elif param.name == 'max_pwm':
                self.max_pwm = param.value
            elif param.name == 'wheel_separation':
                self.wheel_separation = param.value
            elif param.name == 'id':
                self.right_trim = param.value
            elif param.name == 'wheel_diameter':
                self.wheel_separation = param.value
            else:
                raise ValueError(f'unknown parameter {param.name}')
                
        return SetParametersResult(successful=True)
        
    def set_speed(self, left, right):
        """
        Sets the motor speeds between [-1.0, 1.0]
        Override this function for other motor controller setups.
        Should take into account left_trim, right_trim, and max_pwm.
        """
        raise NotImplementedError('MotorController subclasses should implement set_speed()')

    def stop(self):
        self.set_speed(0,0)

    def twist_listener(self, msg):
        self.get_logger().info("Got velocity command : right = %f left = %f"%(msg.right,msg.left))
        self.set_speed(msg.left, msg.right)

    
if __name__ == '__main__':
    raise NotImplementedError("motors.py shouldn't be instantiated directly - instead use motors_nvidia.py, motors_waveshare.py, ect")
    
    
	

