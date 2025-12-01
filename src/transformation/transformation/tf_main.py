from .tf_utils import TFHandler
from .tf_publisher import BlueMarkerPublisher
from .tf_subscriber import MarkerSubscriber
from interfaces.srv import VisionCmd
from interfaces.msg import MarkerData

import cv2
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
import time

class VisionServiceNode(Node):
    def __init__(self, subscriber, publisher, tf_handler):
        super().__init__('vision_service_node')
        self.subscriber = subscriber
        self.publisher = publisher
        self.tf_handler = tf_handler
        
        self.srv = self.create_service(VisionCmd, 'vision_srv', self.service_callback)
        self.get_logger().info("Vision Service 'vision_srv' is ready.")

    def service_callback(self, request, response):
        self.get_logger().info(f"Received Request: '{request.command}'")
        
        if request.command == "scan":
            print("\n--- EXECUTE SCAN (Service Triggered) ---")
            
            # 1. Reset
            self.subscriber.reset()
            
            # 2. Wait for fresh data
            time.sleep(2.5)
            
            # 3. Process
            world_result = self.subscriber.get_world_coordinates(self.tf_handler)
            
            # 4. Print & Publish
            if world_result:
                # --- RESTORED PRINT BLOCK ---
                print("\n=== WORLD COORDINATES RESULTS ===")
                for i, result in enumerate(world_result):
                    pixel_x, pixel_y = result['pixel_center']
                    world_x, world_y, world_z = result['world_coords']
                    print(f"Blue Area {i}:")
                    print(f"  Pixel: ({pixel_x}, {pixel_y})")
                    print(f"  World: X={world_x:.3f}m, Y={world_y:.3f}m, Z={world_z:.3f}m")
                    print()
                # -----------------------------

                self.publisher.publish_blue_markers(world_result)
                print(f"Published {len(world_result)} markers.")
            else:
                print("No markers found.")
                
            self.get_logger().info("Scan complete.")
        
        response.marker_data = MarkerData() 
        return response

def main():
    print("Started tf_main (Service Mode - Integrated)")
    rclpy.init()
    
    subscriber = MarkerSubscriber()
    publisher = BlueMarkerPublisher()
    
    tf_node = rclpy.create_node('tf_handler_node')
    tf_handler = TFHandler(tf_node)
    
    service_node = VisionServiceNode(subscriber, publisher, tf_handler)

    executor = MultiThreadedExecutor()
    executor.add_node(subscriber)
    executor.add_node(tf_node)
    executor.add_node(service_node)
    executor.add_node(publisher)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try: 
        print("Waiting for camera intrinsics...")
        while tf_handler.intrinsics is None:
            time.sleep(0.1)
        print("System Ready. Waiting for service calls...")
        
        while rclpy.ok():
            time.sleep(1)

    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        executor.shutdown()
        publisher.destroy_node()
        subscriber.destroy_node()
        tf_node.destroy_node()
        service_node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()