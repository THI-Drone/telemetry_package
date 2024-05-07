import json
import rclpy
import socket
import os
import select

from rcl_interfaces.msg import Log
from common_package_py.common_node import CommonNode

DEFAULT_UNIX_SOCKET_PATH = "/tmp/thi_drone"

class TelemetryNode(CommonNode):

    def __init__(self):
        # The creation of the needed sockets takes place in the constructor
        super().__init__('telemetry_node')

        if os.path.exists(DEFAULT_UNIX_SOCKET_PATH): 
            print('Socket already exists')
            raise FileExistsError
            
        # Create server socket
        self.server_sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)

        try:
            # Bind server socket and listen for a connection. The node is blocked as long as a connection does not exist
            self.server_sock.bind(DEFAULT_UNIX_SOCKET_PATH)
            self.server_sock.listen(1)

            # Getting the reference for the client socket
            readable, _, _, = select.select([self.server_sock], [], [])
            for sock in readable:
                if sock == self.server_sock:
                    # Accept the connection
                    self.client_sock, _ = self.server_sock.accept()
                    print('connection accepted. Creating Subscription')

                    '''
                    Creating the subscription after the connection has accepted ensures that the callback function is executed only after the soccket is connected
                    '''
                    self.subscription = self.create_subscription(
                        Log,
                        '/rosout',
                        self.subscription_callback, 
                        100)
                    self.subscription

        # In case of exceptions, close the socket and remove its path from disk
        except Exception as e:
            print(f'Error occurred: {e}')
            self.server_sock.close()
            os.remove(DEFAULT_UNIX_SOCKET_PATH)
        
 
    def subscription_callback(self, log_msg):
        try: 
            '''
            Apparently, some characters in the log messages upset something in the json.loads(<received_data>.decode()) function. This is just a brute sanitization of the string, which needs to be implemented in a function of its own
            '''
            log_msg.msg = log_msg.msg.replace(':', '').replace('"', '').replace("'", "").replace("\{", "").replace("}", "")

            self.client_sock.sendall(json.dumps({"type": "std" , "content" :  log_msg.msg}).encode())

        except Exception as e:
            print(f'Error occurred in callback: {e}')


def main(args=None):
    rclpy.init(args=args)

    telemetry_node = TelemetryNode()

    rclpy.spin(telemetry_node)

    telemetry_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
