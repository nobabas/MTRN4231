#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from testing import BlueAreaDetector

class CameraToBlueDetector(Node):
    def __init__(self):
        super().__init__('camera_to_blue_detector')
        self.bridge = CvBridge()
        self.detector = BlueAreaDetector()

        self.subscription = self.create_subscription(
            Image,
            '/camera/raw_file',
            self.listener_callback,
            10)
        
        self.get_logger().info("Listening to /camera/raw_file...")

    def listener_callback(self, msg):
        # Convert the ROS2 Image message to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Set the image inside your BlueAreaDetector
        self.detector.image = frame

        # Run detection
        blue_mask, blue_areas = self.detector.detect_blue_areas(sensitivity=30)

        if not blue_areas:
            self.get_logger().info("No blue areas found in image.")
            return

        marked_image = self.detector.mark_blue_areas(blue_mask)

        cv2.imshow("Blue Areas Detected", marked_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = CameraToBlueDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
