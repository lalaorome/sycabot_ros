import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException

from geometry_msgs.msg import PoseStamped
from sycabot_interfaces.action import Deadzone
from sycabot_interfaces.msg import Motor


class DeadzoneActionClient(Node):

    def __init__(self):
        super().__init__('deadzone_action_client')

        self.declare_parameter('id', 1)

        self.Sycabot_id = self.get_parameter('id').value
        
        self._action_client = ActionClient(self, Deadzone, f'/SycaBot_W{self.Sycabot_id}/deadzones_identification')

    def send_goal(self, order):
        goal_msg = Deadzone.Goal()
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
        self.get_logger().info('Result: {0}'.format(result.deadzones))
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    action_client = DeadzoneActionClient()

    action_client.send_goal(10)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()