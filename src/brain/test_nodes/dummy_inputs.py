#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interfaces.msg import MarkerData

class DummyInputs(Node):
    def __init__(self):
        super().__init__('dummy_inputs')

        # Publisher for blue marker coordinates
        self.marker_pub = self.create_publisher(MarkerData, '/blue_markers_coords', 10)

        # Publish one marker immediately at startup
        marker = MarkerData()
        marker.id = 1.0
        marker.pose = [0.35, 0.12, 0.10, 0.0, 1.57, 0.0]
        self.marker_pub.publish(marker)
        self.get_logger().info("Published dummy marker data: ID 1 at [0.35, 0.12, 0.10]")

def main(args=None):
    rclpy.init(args=args)
    node = DummyInputs()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
