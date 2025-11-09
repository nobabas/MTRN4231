#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interfaces.msg import MarkerData
from interfaces.srv import VisionCmd
from std_msgs.msg import Float32
import random

class DummyInputs(Node):
    def __init__(self):
        super().__init__('dummy_inputs')

        self.marker_pub = self.create_publisher(MarkerData, '/blue_markers_coords', 10)
        self.moisture_pub = self.create_publisher(Float32, '/soil_moisture', 10)
        self.vision_service = self.create_service(VisionCmd, 'vision_srv', self.handle_vision_cmd)

        self.create_timer(2.0, self.publish_marker)
        self.create_timer(1.0, self.publish_moisture)

        self.moisture = 200.0
        self.get_logger().info("DummyInputs started — providing vision_srv + fake topics")

    def handle_vision_cmd(self, request, response):
        self.get_logger().info(f"Received vision command: {request.command}")

        marker = MarkerData()
        marker.id = 1.0                       # ← plain float
        marker.pose = [0.35, 0.12, 0.10]      # ← plain float list

        response.marker_data = marker
        self.get_logger().info(f"Returning fake marker [ID={marker.id}] at {marker.pose}")
        return response

    def publish_marker(self):
        msg = MarkerData()
        msg.id = 1.0
        msg.pose = [0.35, 0.12, 0.10]
        self.marker_pub.publish(msg)
        self.get_logger().info(f"Published dummy marker at {msg.pose}")

    def publish_moisture(self):
        self.moisture += random.uniform(20.0, 50.0)
        msg = Float32()
        msg.data = float(self.moisture)
        self.moisture_pub.publish(msg)
        self.get_logger().info(f"Published dummy soil moisture: {msg.data:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = DummyInputs()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

