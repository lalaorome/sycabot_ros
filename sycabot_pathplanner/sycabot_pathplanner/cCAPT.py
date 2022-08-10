import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException

from sycabot_interfaces.msg import Pose2D
from sycabot_interfaces.action import Control

import numpy as np

class DeadzoneActionClient(Node):

    def __init__(self):
        super().__init__('control_action_client')

        self.declare_parameter('SycaBot_id', 1)

        self.Sycabot_id = self.get_parameter('SycaBot_id').value
        
        self._action_client = ActionClient(self, Control, f'/SycaBot_W{self.Sycabot_id}/MPC_start_control')