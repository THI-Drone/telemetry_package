import json
import rclpy
import socket
import os

from rcl_interfaces.msg import Log
from common_package_py.common_node import CommonNode
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy

DEFAULT_UNIX_SOCKET_PATH = "/tmp/thi_drone"

class TelemetryNode(CommonNode):

    def __init__(self):
        super().__init__('telemetry_node')
        qos_profile = QoSProfile( 
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE, 
            liveliness = QoSLivelinessPolicy.AUTOMATIC
        )

        if os.path.exists(DEFAULT_UNIX_SOCKET_PATH): 
            print('Socket already exists.')
            raise FileExistsError
            
        self.server_sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)

        try:
            self.server_sock.bind(DEFAULT_UNIX_SOCKET_PATH)
            self.server_sock.listen(1)

            # Accept the connection
            self.client_sock, _ = self.server_sock.accept()
            print('Connection accepted. Creating subscription')

            # After connecting, create two subscriptions: one to the '/rosout' topic and one to the 'control' topic
            self.rosout_subscription = self.create_subscription(
                Log,
                '/rosout',
                self.rosout_callback, 
                qos_profile = qos_profile)
            
            self.rosout_subscription
            print('Created subscription to "/rosout".')

            # Commenting this out for test purposes

            # Modify the message type and topic name
            # self.control_subscription = self.create_subscription(
            #     Log,
            #     'control',
            #     self.control_subscription_callback, 
            #     qos_profile = qos_profile)

            # self.control_subscription 
            # print('Created subscription to "control".')

            # Modify the message type and topic name
            self.heartbeat_subscription = self.create_subscription(
               Log,
               'heartbeat', 
                qos_profile = qos_profile)
            
            self.heartbeat_subscription
            print('created subscription to "heartbeat".')

            

        except Exception as e:
            print(f'Error occurred: {e}')
            self.server_sock.close()
            os.remove(DEFAULT_UNIX_SOCKET_PATH)
        


    '''
    @brief callback of the subcriptions. 

    This function takes the content of a log message from the /rosout topic, sanitizes it (altough still quite dirtily) and sends it to the ground station webapp via UNIX socket. 

    @param log_msg the message passed by the topic
    '''
    def rosout_callback(self, log_msg)->None:
        try: 
            #@di-math suggests that this change (#b"\x17") fixes the JSON strings being stitched togeher error 
            self.client_sock.sendall(json.dumps({"type": "std" , "content" :  log_msg.msg}).encode() + b"\x17")
        except Exception as e:
            print(f'Error occurred in rosout_callback: {e}')


    '''
    @brief callback of the subcriptions. 

    This function takes the content of a message from the control topic sanitizes it (altough still quite dirtily) and sends it to the ground station webapp via UNIX socket. 

    @param msg the message passed by the topic
    '''
    def control_callback(self, msg)->None:
        try: 
            self.client_sock.sendall(json.dumps({"type": "std" , "content" :  msg.data}).encode() + b"\x17")
        except Exception as e:
            print(f'Error occurred in control_callback: {e}')


    '''
    @brief destructor function of a CommonNode object

    It cleanly closes the socket and removes the socket path from system
    '''
    def __del__(self):
        print('Destructor called.')
        self.server_sock.close()
        os.remove(DEFAULT_UNIX_SOCKET_PATH)


def main(args=None):
    rclpy.init(args=args)

    telemetry_node = TelemetryNode()

    rclpy.spin(telemetry_node)

    telemetry_node.destroy_node()
    rclpy.on_shutdown(telemetry_node.__del__())


if __name__ == '__main__':
    main()
