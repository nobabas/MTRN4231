#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
import os

class ImageSaverSubscriber(Node):
    def __init__(self):
        super().__init__('image_saver_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/raw_file',
            self.listener_callback,
            10)
        self.subscription
        self.get_logger().info("Image saver node started. Waiting for messages...")

        # Directory to save incoming images
        self.save_dir = "/home/mtrn/4231/received_images"
        os.makedirs(self.save_dir, exist_ok=True)

    def listener_callback(self, msg: Image):
        self.get_logger().info(f"Received image: {msg.width}x{msg.height}")

        # Convert ROS Image message → OpenCV image (numpy array)
        frame = np.frombuffer(msg.data, dtype=np.uint8)
        frame = frame.reshape((msg.height, msg.width, 3))

        # Generate a filename with timestamp
        filename = os.path.join(
            self.save_dir,
            f"current_image.jpg"
        )

        # Save the image
        success = cv2.imwrite(filename, frame)
        if success:
            self.get_logger().info(f"Saved image to {filename}")
        else:
            self.get_logger().error(f"Failed to save image to {filename}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaverSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
