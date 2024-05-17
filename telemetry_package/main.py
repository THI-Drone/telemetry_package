import json
import rclpy
import socket
import os
import sys

from rcl_interfaces.msg import Log
from common_package_py.common_node import CommonNode
from common_package_py.topic_names import TopicNames

from interfaces.msg import Control, Heartbeat
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy

DEFAULT_UNIX_SOCKET_PATH = "/tmp/thi_drone"

class TelemetryNode(CommonNode):
    def __init__(self):
        '''
        Constructor function of the TelemetryNode class. 

        This function firstly provides the necessary QoS setting for each subscription responsible for sending data to the ground-station webapp.  

        It then builds the UNIX socket used for the communication with the ground-station frontend and makes the node listen for a connection. The system will raise a warning if the socket is already in use. 

        After building the UNIX socket, this function creates three subscription: one to /rosout, one to TopicNames.Control and one to TopicNames.Heartbeat.  

        Args: 
            None
        
        Returns: 
            None
        '''

        super().__init__('telemetry_node')
        qos_profile = QoSProfile( 
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=15,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE, 
            liveliness = QoSLivelinessPolicy.AUTOMATIC
        )

        self.chosen_socket_path = ''

        if sys.argv[1] == '': self.chosen_socket_path = DEFAULT_UNIX_SOCKET_PATH
        else: self.chosen_socket_path = sys.argv[1]


        if os.path.exists(self.chosen_socket_path): 
            raise FileExistsError('Socket already exists.')
            
        self.server_sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)

        try:
            self.server_sock.bind(self.chosen_socket_path)
            self.server_sock.listen(1)

            # Accept the connection
            self.client_sock, _ = self.server_sock.accept()
            print('Connection accepted. Creating subscription')


            self.rosout_subscription = self.create_subscription(
                Log,
                '/rosout',
                self.__rosout_callback, 
                qos_profile = qos_profile)
            
            self.rosout_subscription
            print('Created subscription to "/rosout".')


            self.control_subscription = self.create_subscription(
                Control,
                TopicNames.Control,
                qos_profile = qos_profile)

            self.control_subscription 
            print('Created subscription to "control".')


            self.heartbeat_subscription = self.create_subscription(
                Heartbeat,
                TopicNames.Heartbeat, 
                self.__heartbeat_callback,
                qos_profile = qos_profile)
            
            self.heartbeat_subscription
            print('created subscription to "heartbeat".')

            # Deactivating the node as according to the docs, could result in an error? (unsure yet)
            self._deactivate_()

        except Exception as e:
            print(f'Error occurred: {e}')
            self.server_sock.close()
            os.remove(self.chosen_socket_path)
        

    def __rosout_callback(self, log_msg)->None:
        '''
        Callback function for the /rosout subscription. 

        This function takes each message that is NOT of the severity level DEBUG and sends it tho the ground-station webapp via UNIX socket.

        Args: 
            log_msg: the incoming log message from the /rosout topic
        
            Returns: 
                None
        '''

        if log_msg.level != 'DEBUG':  
            try: 
                self.client_sock.sendall(json.dumps(
                    {"type": "std" ,
                    "content" : {
                        "level" : log_msg.level, 
                        "name" : log_msg.name, 
                        "msg" : log_msg.msg     
                            }}  ).encode() + b"\x17")
            except Exception as e:
                print(f'Error occurred in rosout_callback: {e}')


    def __heartbeat_callback(self, hb_msg)->None:
        '''
        Callback function for the TopicNames.Heartbeat subscription. 

        This function takes each message coming from TOpicNames.Heartbeat and sends it tho the ground-station webapp via UNIX socket.

        Args: 
            hb_msg: the incoming log message from the /rosout topic
        
            Returns: 
                None
        '''

        try: 
            self.client_sock.sendall(json.dumps(
                {"type": "std" ,
                "content" : {
                    "sender_id" : hb_msg.sender_id, 
                    "tick" : hb_msg.tick, 
                    "active" : hb_msg.active, 
                    "timestamp" : hb_msg.time_stamp     
                        }}  ).encode() + b"\x17")
        except Exception as e:
            print(f'Error occurred in heartbeat_callback: {e}')


    def __del__(self):
        ''' 
        Destructor function of the TelemetryNode class. 

        This function ensures that the server is closed cleanly and removed from disk. 

        Args: 
            None
        
        Returns:
            None 
        '''

        print('Destructor called.')
        self.server_sock.close()
        os.remove(self.chosen_socket_path)


def main(args=None):
    rclpy.init(args=args)

    telemetry_node = TelemetryNode()

    rclpy.spin(telemetry_node)

    telemetry_node.destroy_node()

    # Ensures that the destructor function of the TelemetryNode object is executed on program shutdown
    rclpy.on_shutdown(telemetry_node.__del__())


if __name__ == '__main__':
    main()
