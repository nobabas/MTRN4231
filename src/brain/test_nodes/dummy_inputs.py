#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class DummyMoisture(Node):
    def __init__(self):
        super().__init__('dummy_moisture')

        # Publisher for soil moisture topic
        self.moisture_pub = self.create_publisher(Float32, '/soil_moisture', 10)

        # Publish every second
        self.create_timer(1.0, self.publish_moisture)

        # Start with low initial value
        self.moisture = 200.0
        self.get_logger().info("DummyMoisture started — publishing fake soil moisture readings")

    def publish_moisture(self):
        # Simulate rising soil moisture until ~750
        if self.moisture < 750:
            self.moisture += random.uniform(20.0, 40.0)
        else:
            # Reset periodically to simulate dry soil again
            self.moisture = 200.0

        msg = Float32()
        msg.data = float(self.moisture)
        self.moisture_pub.publish(msg)
        self.get_logger().info(f"Published dummy soil moisture: {msg.data:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = DummyMoisture()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
