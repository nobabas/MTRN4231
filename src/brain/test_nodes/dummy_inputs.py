#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interfaces.msg import MarkerData
import random
import time

class DummyInputs(Node):
    def __init__(self):
        super().__init__('dummy_inputs')

        # Only publish marker data now
        self.marker_pub = self.create_publisher(MarkerData, '/blue_markers_coords', 10)

        # Publish every 3 seconds
        self.timer = self.create_timer(3.0, self.publish_marker)
        self.get_logger().info("DummyInputs node started (publishing only marker data).")

    def publish_marker(self):
        msg = MarkerData()
        msg.id = float(random.randint(1, 3))
        msg.pose = [
            round(random.uniform(0.3, 0.5), 2),
            round(random.uniform(0.1, 0.2), 2),
            round(random.uniform(0.05, 0.15), 2)
        ]
        self.marker_pub.publish(msg)
        self.get_logger().info(f"Published dummy marker: ID {msg.id} at {msg.pose}")

def main(args=None):
    rclpy.init(args=args)
    node = DummyInputs()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

