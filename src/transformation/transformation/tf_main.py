from .tf_utils import TFHandler
from .tf_publisher import BlueMarkerPublisher
from .tf_subscriber import MarkerSubscriber

import cv2
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
import time


def main():
    print("Started tf_main")
    rclpy.init()
    subscriber = MarkerSubscriber()
    publisher = BlueMarkerPublisher()
    node = rclpy.create_node('tf_handler_node')
    tf_handler = TFHandler(node)
    
    # ADDED MAKE SURE TO CHANGE 
    # tf_handler.set_mock_intrinsics()

    executor = MultiThreadedExecutor()
    executor.add_node(subscriber)
    executor.add_node(node)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try: 
        # Enable mock intrinsics if needed (optional)
        # if tf_handler.intrinsics is None:
        #        tf_handler.set_mock_intrinsics()

        print("Waiting for camera intrinsics...")
        
        # 4. Wait for intrinsics (The background executor handles the callbacks)
        while tf_handler.intrinsics is None:
            time.sleep(0.1)

        print("Camera intrinsics received!")
        print("Waiting for ROS 2 node discovery...")
        time.sleep(2)
        
        print("Subscriber and TF Handler are now active...")
        print("Waiting 5 seconds for all markers to arrive...")
        
        # 5. Wait for markers to be collected
        # The executor is still spinning in the background, so:
        # - subscriber is collecting markers
        # - tf_handler is updating self.current_depth
        time.sleep(5.0) 
                   
        print(f"Successfully received {len(subscriber.blue_area)} markers!")
        
        world_result = subscriber.get_world_coordinates(tf_handler)
        # world_result = world_result.transform_camera_to_world1(world_result)
        print("Waiting 5 seconds for brain_node to connect...")
        time.sleep(5.0)  # Wait for 5 seconds

        publisher.publish_blue_markers(world_result)

        print("\n=== WORLD COORDINATES RESULTS ===")
        for i, result in enumerate(world_result):
            pixel_x, pixel_y = result['pixel_center']
            world_x, world_y, world_z = result['world_coords']
            print(f"Blue Area {i}:")
            print(f"  Pixel: ({pixel_x}, {pixel_y})")
            print(f"  World: X={world_x:.3f}m, Y={world_y:.3f}m, Z={world_z:.3f}m")
            print()
    
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        executor.shutdown()
        publisher.destroy_node()
        subscriber.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
