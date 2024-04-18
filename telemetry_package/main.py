import sys
import rclpy
from rclpy.node import Node

from common_package_py.common_node import CommonNode


class TelemetryNode(CommonNode):
    """A class that represents a telemetry node.
    """
    
    def __init__(self, id: str):
        """Creates a new TelemetryNode.

        Args:
            id (str): Unique node id
        """
        
        super().__init__(id)


def main(args=None):
    rclpy.init(args=args)

    telemetry_node = TelemetryNode("telemetry_node")

    rclpy.spin(telemetry_node)

    telemetry_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
