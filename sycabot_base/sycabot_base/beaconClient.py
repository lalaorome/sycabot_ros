import rclpy
from rclpy.node import Node
from sycabot_interfaces.msg import BeaconMsg
from rclpy.qos import qos_profile_sensor_data



SYCABOT_ID = 1
verbose = True
plot = True

class beaconClient(Node):
    '''
    Node responsable for advertising the Sycabot id to the central PC
    by publishing on the beacon topic.
    '''
    def __init__(self):
        super().__init__('BeaconClient')
        qos = qos_profile_sensor_data
        self.declare_parameter('id', 1)
        self.id = self.get_parameter('id').value

        self.beacon = self.create_publisher(BeaconMsg, '/beacon', qos)
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
    