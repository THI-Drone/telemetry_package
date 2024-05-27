import pytest 
import rclpy
import sys
import unittest
from rclpy.executors import SingleThreadedExecutor
from rclpy.time import Time
from main import TelemetryNode

"""
The objective of this file is testing various aspects of the GS functionality. These include: 

    - Performance 
    - Resiliance to wrongful messages  
    - Resistance to possible sabotage
        - Tempering with the sockets 
        - Wrongful Socket path -> DONE
        - Unknown Node?
    - Unknown messages maybe idk
"""
class TelemetryNodeTests(unittest.TestCase):
    def test_cli_arguments_validity(self):
        """
        This function tests the valiity of the CLI arguments (which are used to specify a path for the UNIX socket) passed to the node. 
        """
        rclpy.init()
        telemetry_node = TelemetryNode('telemetry')
        test_node = rclpy.create_node('test')

        executor = SingleThreadedExecutor()

        shutdown_timer_duration = 10.0

        def valid_socket_input_callback(): 
            """
            How to test socket arguments? 
            
            Set the system arguments to different outputs 
            Run the constructor of telemetry_node
            """
            sys.argv[1] = '/tmp/test_socket'
            self.assertRaises(None, telemetry_node.__init__)

            sys.argv[1] = 12345
            self.assertRaises(TypeError, telemetry_node.__init__)

            sys.argv[1] = True
            self.assertRaises(TypeError, telemetry_node.__init__)

        def end_timer_callback(): 
            executor.shutdown(0)

        valid_input_runner = test_node.create_timer(
            0, valid_socket_input_callback
        )

        timeout_timer = test_node.create_timer(
            shutdown_timer_duration, end_timer_callback
        )

        executor.add_node(telemetry_node)
        executor.add_node(test_node)

        executor.spin()
        del executor

def test_performance(self):
    """
    This function tests the node's performance by measuring the rate at which it sends messages. 
    """
    telemetry_node = TelemetryNode('telemetry')
    test_node = rclpy.create_node('test')

    executor = SingleThreadedExecutor()

    shutdown_timer_duration = 10.0

    def input_msgs_test_callback(): 
        """
        How to test performance?


        Set up an UNIX-socket using the callback function and keep updating the message rate for a determined amount of time (60s)
        """


    def end_timer_callback(): 
        executor.shutdown(0)

    valid_input_runner = test_node.create_timer(
        0, input_msgs_test_callback
    )

    timeout_timer = test_node.create_timer(
        shutdown_timer_duration, end_timer_callback
    )
    
    executor.add_node(telemetry_node)
    executor.add_node(test_node)

    executor.spin()
    del executor

if __name__ == '__main__': 
    pytest.main()