#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import os

class ImageFilePublisher(Node):
    def __init__(self):
        super().__init__('image_file_publisher')
        pathing = '/camera/raw_file'
        self.publisher = self.create_publisher(Image, pathing, 10)

        # Image path - update this to your image
        image_path = "/home/mtrn/4231/testing/test.png"

        if not os.path.exists(image_path):
            self.get_logger().error(f"Image not found: {image_path}")
            return

        frame = cv2.imread(image_path)
        if frame is None:
            self.get_logger().error("Failed to load image file.")
            return

        # Convert OpenCV image → ROS2 Image message manually
        height, width, channels = frame.shape
        msg = Image()
        msg.height = height
        msg.width = width
        msg.encoding = 'bgr8'
        msg.step = width * channels
        msg.data = frame.tobytes()
        msg.header.frame_id = "camera_frame"
        msg.header.stamp = self.get_clock().now().to_msg()

        # Publish the image
        self.publisher.publish(msg)
        self.get_logger().info(f"Published {image_path} to /camera/raw_file")
        self.get_logger().info(f"Image size: {width}x{height}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageFilePublisher()
    # Give time for the message to be published
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()