# PublishFinal.py
# Combines blue detection and TF utilities

import cv2
import rclpy
from rclpy.node import Node
from interfaces.msg import MarkerData  
from blue_detector.bluedetector import BlueAreaDetector
from blue_detector.tf_utils import TFHandler


class BlueMarkerPublisher(Node):
    """Publishes world coordinates of detected blue markers."""

    def __init__(self):
        super().__init__('blue_marker_publisher')
        self.marker_publisher = self.create_publisher(
            MarkerData,
            '/blue_markers_coords',
            10
        )

    def publish_blue_markers(self, world_result):
        for i, result in enumerate(world_result):
            world_x, world_y, world_z = result['world_coords']

            msg = MarkerData()
            msg.id = cfloat(i)
            msg.pose = [cfloat(world_x), cfloat(world_y), 0.1, 0.0, 0.0, 0.0]

            self.marker_publisher.publish(msg)
            self.get_logger().info(
                f"Published marker_{i} at [{world_x:.2f}, {world_y:.2f}]"
            )


def main(args=None):
    """Run blue detection, transform coordinates, and publish marker messages."""
    rclpy.init(args=args)

    detector = BlueAreaDetector()
    publisher = BlueMarkerPublisher()

    try:
        if detector.load_image_file():
            detector.image = cv2.cvtColor(detector.image, cv2.COLOR_BGR2RGB)
            blue_mask, _ = detector.detect_blue_areas()
            marked_image = detector.mark_blue_areas(blue_mask)

            cv2.imshow("Blue Areas Detected", marked_image)
            cv2.imshow("Blue Mask", blue_mask)
            print(detector.generate_report())

        # Create temporary node for TF utilities
        node = rclpy.create_node('tf_handler_node')
        tf_handler = TFHandler(node)

        if tf_handler.intrinsics is None:
            tf_handler.set_mock_intrinsics()

        rclpy.spin_once(node, timeout_sec=2.0)

        world_result = detector.get_world_coordinates(tf_handler, depth_value=1000)
        publisher.publish_blue_markers(world_result)

        print("\n=== WORLD COORDINATES RESULTS ===")
        for i, result in enumerate(world_result):
            pixel_x, pixel_y = result['pixel_center']
            world_x, world_y, world_z = result['world_coords']
            print(f"Blue Area {i+1}:")
            print(f"  Pixel: ({pixel_x}, {pixel_y})")
            print(f"  World: X={world_x:.3f} m, Y={world_y:.3f} m, Z={world_z:.3f} m")
            print()

    except KeyboardInterrupt:
        print("Shutting down...")

    finally:
        publisher.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
