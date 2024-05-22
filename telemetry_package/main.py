import json
import rclpy
import socket
import os
import sys
import time

from rcl_interfaces.msg import Log
import rclpy.logging
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy
from common_package_py.topic_names import TopicNames
from interfaces.msg import Heartbeat

DEFAULT_UNIX_SOCKET_PATH = "/tmp/thi_drone"


class TelemetryNode(Node):

    LOG_LEVELS = {
        rclpy.logging.LoggingSeverity.DEBUG: 'DEBUG',
        rclpy.logging.LoggingSeverity.INFO: 'INFO',
        rclpy.logging.LoggingSeverity.WARN: 'WARN',
        rclpy.logging.LoggingSeverity.ERROR: 'ERROR',
        rclpy.logging.LoggingSeverity.FATAL: 'FATAL',
    }

    # Defines the minimum time in ms that has to pass, before the next log-message with this name will be send to the UNIX-socket. All messages in between will not be send.
    THROTTLE_LOG_MESSAGE_DEFINITION = {
        "fcc_bridge.fcc_telemetry": 500,
        "fcc_bridge.safety": 1000
    }

    def __init__(self):
        """
        Constructor function of the TelemetryNode class.

        This function firstly provides the necessary QoS setting for each subscription, responsible for sending data to the ground-station webapp.

        It then builds the UNIX socket used for the communication with the ground-station frontend and makes the node listen for a connection. The system will raise a warning if the socket is already in use.

        After building the UNIX socket, this function creates two subscription: /rosout and TopicNames.Heartbeat.

        Args:
            None

        Returns:
            None
        """

        super().__init__('telemetry_node')

        # Default QoS for the GroundStation - Node
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=15,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            liveliness=QoSLivelinessPolicy.AUTOMATIC
        )

        # For tracking the timestamps between the last received messages for throttled log-message names
        self._message_throttle_table = {}

        # Assume second and only arg as explicit path to unix socket
        if len(sys.argv) > 1:
            self.chosen_socket_path = sys.argv[1]
        else:
            self.chosen_socket_path = DEFAULT_UNIX_SOCKET_PATH

        # Check if the socket already exists
        if os.path.exists(self.chosen_socket_path):
            raise FileExistsError('Socket already exists.')

        try:
            # Create the socket on the given path and listen for one connection
            self.server_sock = socket.socket(
                socket.AF_UNIX, socket.SOCK_STREAM)
            self.server_sock.bind(self.chosen_socket_path)
            print(
                f"Listening for connection on the UNIX-socket at {self.chosen_socket_path} ...")
            self.server_sock.listen(1)

            # Accept the connection
            self.client_sock, _ = self.server_sock.accept()
            print('Connection accepted. Creating subscriptions...')

            # Subscribe to /rosout
            self.rosout_subscription = self.create_subscription(
                Log,
                '/rosout',
                self.__rosout_callback,
                qos_profile=qos_profile)

            self.rosout_subscription
            print('Created subscription to "/rosout".')

            # Subscribe to TopicNames.Heartbeat
            self.heartbeat_subscription = self.create_subscription(
                Heartbeat,
                TopicNames.Heartbeat,
                self.__heartbeat_callback,
                qos_profile=qos_profile)

            self.heartbeat_subscription
            print('created subscription to "heartbeat".')

        except Exception as e:
            print(f'Error occurred: {e}')
            self.server_sock.close()
            os.remove(self.chosen_socket_path)

    def throttle_log_message(self, log_msg) -> bool:
        """
        Using self.THROTTLE_LOG_MESSAGE_DEFINITION and self._message_throttle_table, this function return a bool based on, if a message should be send or not with regard to the minimum time to pass between to subsequent messages
        defined in self.THROTTLE_LOG_MESSAGE_DEFINITION.

        Args:
            log_msg: the incoming log message from the /rosout topic

        Returns:
            True, if message should be throttled (not send). False, if the message can be send.

        """
        if log_msg.name in self.THROTTLE_LOG_MESSAGE_DEFINITION:
            if log_msg.name in self._message_throttle_table and (time.time() - self._message_throttle_table[log_msg.name])*1000 < self.THROTTLE_LOG_MESSAGE_DEFINITION[log_msg.name]:
                return True
            else:
                self._message_throttle_table[log_msg.name] = time.time()
                return False
        else:
            return False

    def __rosout_callback(self, log_msg) -> None:
        """
        Callback function for the /rosout subscription.

        This function takes each message that is NOT of the severity level DEBUG and sends it tho the ground-station webapp via UNIX socket.

        Args:
            log_msg: the incoming log message from the /rosout topic

        Returns:
            None
        """

        # Filter log messages
        # Don't log DEBUG level messages
        if log_msg.level == rclpy.logging.LoggingSeverity.DEBUG:
            return
        # Check if message should be throttled
        log_is_higher_severity = log_msg.level == rclpy.logging.LoggingSeverity.WARN or log_msg.level == rclpy.logging.LoggingSeverity.ERROR or log_msg.level == rclpy.logging.LoggingSeverity.FATAL
        if not log_is_higher_severity and self.throttle_log_message(log_msg):
            return

        try:
            self.client_sock.sendall(json.dumps(
                {"type": "log",
                    "content": {
                        "level": self.LOG_LEVELS[log_msg.level],
                        "name": log_msg.name,
                        "msg": log_msg.msg
                    }}).encode() + b"\x17")
        except Exception as e:
            print(f'Error occurred in rosout_callback: {e}')

    def __heartbeat_callback(self, hb_msg) -> None:
        """
        Callback function for the TopicNames.Heartbeat subscription. 

        This function takes each message coming from TopicNames.Heartbeat and sends it to the ground-station webapp via UNIX socket.

        Args: 
            hb_msg: the incoming log message from the /rosout topic

        Returns: 
            None
        """

        try:
            self.client_sock.sendall(json.dumps(
                {"type": "heartbeat",
                 "content": {
                     "sender_id": hb_msg.sender_id,
                     "tick": hb_msg.tick,
                     "active": hb_msg.active,
                     "timestamp": hb_msg.time_stamp.sec + hb_msg.time_stamp.nanosec / 1e9
                 }}).encode() + b"\x17")
        except Exception as e:
            print(f'Error occurred in heartbeat_callback: {e}')

    def __del__(self):
        """ 
        Destructor function of the TelemetryNode class. 

        This function ensures that the server is closed cleanly and removed from disk. 

        Args: 
            None

        Returns:
            None 
        """

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
