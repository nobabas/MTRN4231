import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime
from pathlib import Path  
import numpy as np


class ImageSaverSubscriber(Node):
    def __init__(self):
        super().__init__('image_saver_subscriber')
        self.subscription = self.create_subscription(
            Image,
            #Changed based on camera topic on lab computer
            '/camera/camera/color/image_raw',
            self.listener_callback,
            10)
        self.get_logger().info("Image saver node started. Waiting for one image...")

        # Directory to save incoming images
        # Change to where it's needed
        self.save_dir = "/home/mtrn/4231/received_images"
        os.makedirs(self.save_dir, exist_ok=True)

        # Flag to ensure we only save once
        self.image_saved = False

    def listener_callback(self, msg: Image):
        if self.image_saved:
            return  # Ignore any extra images

        self.get_logger().info(f"Received image: {msg.width}x{msg.height}")

        # Convert ROS Image message → OpenCV image (numpy array)
        frame = np.frombuffer(msg.data, dtype=np.uint8)
        frame = frame.reshape((msg.height, msg.width, 3))
        
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Generate a filename
        filename = os.path.join(self.save_dir, "current_image.jpg")

        # Save the image
        success = cv2.imwrite(filename, frame_rgb)
        if success:
            self.get_logger().info(f"Saved image to {filename}")
            self.image_saved = True
            # Shut down after saving the image
            self.get_logger().info("Shutting down after saving one image.")
            rclpy.shutdown()
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

if __name__ == '__main__':
    main()
