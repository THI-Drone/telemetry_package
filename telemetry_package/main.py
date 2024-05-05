'''
This code gets too many messages all at once. Hence it needs to: 
    - The Node only sends the first message it receives
'''


import json
import os
import select
import socket
# import sys
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log


DEFAULT_UNIX_SOCKET_PATH = "/tmp/thi_drone"


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('telemetry_node')
        self.subscription = self.create_subscription(
            Log,
            '/rosout',
            self.run_server, 
            100)
        self.subscription 

        # Counter for debugging purposes
        self.msg_cnt = 0

        self.msg_cache = list()
        self.cache_size = 400


    def is_socket_closed(self, sock: socket.socket) -> bool:
        try:
            # this will try to read bytes without blocking and also without removing them from buffer (peek only)
            data = sock.recv(16, socket.MSG_DONTWAIT | socket.MSG_PEEK)
            if len(data) == 0:
                return True
        except BlockingIOError:
            return False  # socket is open and reading from it would block
        except ConnectionResetError:
            return True  # socket was closed for some other reason
        except Exception as e:
            self.get_logger().error(f"Unexpected exception when checking if the socket is closed: {e}")
            return False
        return False
    

    '''
    Code to implement the websocket functionality     
    '''
    def socket_exists(self, socket_path: str) -> bool:
        return os.path.exists(socket_path)


    def send_message(self, client_socket):
        """
        Defines the message block to be continuosly send.
        """
        if len(self.msg_cache) > 0:
            client_socket.sendall(json.dumps(self.msg_cache[0]).encode())
            self.get_logger().info(f'Send message #{self.msg_cnt + 1}')
            self.msg_cnt += 1     
            self.msg_cache.pop(0)  


    def run_server(self, msg, socket_path=DEFAULT_UNIX_SOCKET_PATH):
        # Check if socket is in use
        if self.socket_exists(socket_path):
            self.get_logger().fatal(f'Socket path already exists')
            raise FileExistsError('')

        # Create a socket
        server_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)

        try:
            # Bind socket
            server_socket.bind(socket_path)
            self.get_logger().info(f'Binding socket.')

            # Listen for connection
            server_socket.listen(1)
            self.get_logger().info(f'Listening for connection on {socket_path} "...')

            while True:
                # Appending the stream of sent messages so that the socket can take that amount of traffic
                self.msg_cache.append(msg.data)
                self.get_logger().info(f'Received message with following payload: {self.msg_cache[len(self.msg_cache) - 1]}')

                if len(self.msg_cache) > self.cache_size: 
                    self.msg_cache.pop(0)


                readable, _, _, = select.select([server_socket], [], [])
                for sock in readable:
                    if sock == server_socket:
                        # Accept the connection
                        client_socket, _ = server_socket.accept()
                        self.get_logger().info(f'Connection accepted')

                        try:
                            while True:
                                self.get_logger().info(f'Sending the following message through websocket: {msg.data}')
                                self.send_message(client_socket)

                        except Exception as e:
                            self.get_logger().error(f'Error sending data, {e}')

                        finally:
                            # Close client socket
                            self.get_logger().info(f'Closing socket.')
                            client_socket.close()

        except Exception as e:
            self.get_logger().fatal(f'The following error eccured before the socket could be bound: {e}')

        finally:
            server_socket.close()
            self.get_logger().info(f'Closing socket and removing {DEFAULT_UNIX_SOCKET_PATH}.')
            os.remove(socket_path)


def main(args=None):
    rclpy.init(args=args)

    telemetry_node = MinimalPublisher()

    rclpy.spin(telemetry_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    telemetry_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
