import rclpy
from rclpy.node import Node

from sycabot_interfaces.msg import BeaconMsg


SYCABOT_ID = 1
verbose = True
plot = True

class beaconClient(Node):
    def __init__(self):
        super().__init__('BeaconClient')
        self.declare_parameter('id', 1)
        self.id = self.get_parameter('id').value

        self.beacon = self.create_publisher(BeaconMsg, 'beacon', 10)
        self.timer = self.create_timer(1., self.pub_beacon_cb)

    def pub_beacon_cb(self):
        data = BeaconMsg()
        data.id = self.id
        self.beacon.publish(data)
        

def main(args=None):

    rclpy.init(args=args)
    jb = beaconClient()
    rclpy.spin(jb)
    return
    


if __name__ == '__main__':
    main()
    