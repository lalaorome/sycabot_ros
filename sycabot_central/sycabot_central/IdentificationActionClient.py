import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException

from geometry_msgs.msg import PoseStamped
from sycabot_interfaces.action import Identification
from sycabot_interfaces.msg import Motor


class DeadzoneActionClient(Node):

    def __init__(self, n_inputs, types):
        super().__init__('identification_action_client')
        
        self.declare_parameter('SycaBot_id', 1)

        self.Sycabot_id = self.get_parameter('SycaBot_id').value

        self.n_inputs = n_inputs
        self.types = types

        self._action_client = ActionClient(self, Identification, f'/SycaBot_W{self.Sycabot_id}/start_identification')

    def send_goal(self, order):
        goal_msg = Identification.Goal()
        goal_msg.n_inputs = self.n_inputs
        goal_msg.types = self.types
        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.success}')
        rclpy.shutdown()


def main(args=None):
    n_inputs = input("Enter number of inputs : ")
    if n_inputs == 'test' :
        types = ['ramp']
        n_inputs = 100
    else :
        n_inputs = int(n_inputs)
        nb_types = int(input("Enter number of types of input : "))
        types = []
        accepted_types = ['ramp', 'random_sine_cosine', 'random_sine', 'step']
        result = False
        for i in range(nb_types):
            result = False
            while not result :
                types.append(str(input(f"Enter type num. {i+1} : ")))
                result = any( types[-1]== t for t in accepted_types)
                if not result :
                    types.pop()
                    print('Wrong input type.')
        
    rclpy.init(args=args)

    action_client = DeadzoneActionClient(n_inputs, types)

    action_client.send_goal(10)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()