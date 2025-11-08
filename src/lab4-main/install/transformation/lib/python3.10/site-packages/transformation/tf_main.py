from .tf_utils import TFHandler
from .tf_publisher import BlueMarkerPublisher
from .tf_subscriber import MarkerSubscriber

import cv2
import rclpy
from rclpy.node import Node

def main():
    rclpy.init()
    subscriber = MarkerSubscriber()
    publisher = BlueMarkerPublisher()
    rclpy.spin_once()
    try: 
        node = rclpy.create_node('test_tf')
        tf_handler = TFHandler(node)
        if tf_handler.intrinsics is None:
                tf_handler.set_mock_intrinsics()

        rclpy.spin_once(node, timeout_sec=2.0)

        world_result = subscriber.get_world_coordinates(tf_handler, depth_value=1000)
        publisher.publish_blue_markers(world_result)

        print("\n=== WORLD COORDINATES RESULTS ===")
        for i, result in enumerate(world_result):
            pixel_x, pixel_y = result['pixel_center']
            world_x, world_y, world_z = result['world_coords']
            print(f"Blue Area {i+1}:")
            print(f"  Pixel: ({pixel_x}, {pixel_y})")
            print(f"  World: X={world_x:.3f}m, Y={world_y:.3f}m, Z={world_z:.3f}m")
            print()
    
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        # cv2.waitKey(0)
        publisher.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
