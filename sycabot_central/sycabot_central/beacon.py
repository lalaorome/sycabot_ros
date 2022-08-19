import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from sycabot_interfaces.msg import BeaconMsg
from sycabot_interfaces.srv import BeaconSrv
from std_srvs.srv import Trigger
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor

class beacon(Node):
    '''
    A beacon is responsible for doing the link between the robots and any user by collecting the ids of the SycaBots that are publishing on the beacon topic.
    It is possible to refresh the list if it is not updated.
    '''
    def __init__(self):
        super().__init__('beacon')
        qos = qos_profile_sensor_data
        self.ids = []
        self.prev_count = 0
        cb_group = ReentrantCallbackGroup()
        self.beacon_sub = self.create_subscription(BeaconMsg, 'beacon',  self.get_jetbot_ids_cb, qos)
        self.send_ids_srv = self.create_service(BeaconSrv, 'get_list_ids', self.send_ids_cb)
        self.refresh_ids = self.create_service(Trigger, 'refresh_list_ids', self.refresh_cb)
        self.check_updated = self.create_timer(0.01, self.check_updated_cb, callback_group = cb_group)
    
    def get_jetbot_ids_cb(self, msg):
        '''
        Get the IDs for each jetbot advertising on this topic.

        arguments :
        ------------------------------------------------
        return :
        '''
        print(msg.id)
        if msg.id not in self.ids :
            self.ids.append(msg.id)
            self.ids.sort()        
        self.get_logger().info('ids : ' + str(self.ids))
    
    def refresh_cb(self, request, response):
        '''
        Callback for the refresh_list_ids service.
        Refresh the IDs when asked by a client.

        arguments :
        ------------------------------------------------
        return :
        '''
        self.ids = []
        response.success = True
        return response
    
    def send_ids_cb(self, request, response):
        '''
        Callback for the get_list_ids.
        Sends the IDs when requested by a client.

        arguments :
        ------------------------------------------------
        return :
        '''
        if self.updated :
            response.success = True
            response.ids = self.ids
            print(self.ids)
        else :
            response.success = False
            response.message = 'Not yet udpated. Try to refresh.'
        return response

    def check_updated_cb(self):
        '''
        Callback for the check_updated timer.
        Check if the beacon node is up to date.

        arguments :
        ------------------------------------------------
        return :
        '''
        if len(self.ids) == self.count_publishers('/beacon') : self.updated = True
        else : self.updated = False



def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = beacon()
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
