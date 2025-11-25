from .tf_utils import TFHandler
from .tf_publisher import BlueMarkerPublisher
from .tf_subscriber import MarkerSubscriber

import cv2
import rclpy
from rclpy.node import Node
import threading
import time

def main():
    rclpy.init()
    subscriber = MarkerSubscriber()
    publisher = BlueMarkerPublisher()
    
    try: 
        node = rclpy.create_node('test_tf')
        tf_handler = TFHandler(node)
        # Enable mock intrinsics 
        if tf_handler.intrinsics is None:
               tf_handler.set_mock_intrinsics()
        # Wait until camera intrinsics are received
        while tf_handler.intrinsics is None:
            rclpy.spin_once(tf_handler.node, timeout_sec=0.1)
            tf_handler.node.get_logger().warn("Waiting for camera intrinsics...")


        print("Waiting for ROS 2 node discovery...")
        time.sleep(2)

        def spin_node():
            rclpy.spin(subscriber)
            
        spin_thread = threading.Thread(target=spin_node, daemon=True)
        spin_thread.start()
        
        print("Subscriber is now actively listening...")
        print("Waiting 5 seconds for all markers to arrive...")
        
        # Give the subscriber 5s to collect all messages
        time.sleep(5.0) 
                   
        print(f"Successfully received {len(subscriber.blue_area)} markers!")
        

        world_result = subscriber.get_world_coordinates(tf_handler, depth_value=400)
        # world_result = world_result.transform_camera_to_world1(world_result)
        print("Waiting 5 seconds for brain_node to connect...")
        time.sleep(5.0)  # Wait for 5 seconds

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
        publisher.destroy_node()
        subscriber.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
