#This will combine testing and tf_utils.py files
from testing import BlueAreaDetector
from tf_utils import TFHandler

import cv2
import rclpy
#from interface import MarkerData
from std_msgs.msg import Header
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class BlueMarkerPublisher(Node):
    def __init__(self):
        super().__init__('blue_marker_publisher')
        
        # Publish as Float32MultiArray messages
        self.marker_publisher = self.create_publisher(
            Float32MultiArray,  # Changed this
            '/blue_markers_coords', 
            10
        )
    
    def publish_blue_markers(self, world_result):
        for i, result in enumerate(world_result):
            world_x, world_y, world_z = result['world_coords']
            
            # Create Float32MultiArray message correctly
            array_msg = Float32MultiArray()  # Added parentheses
            #array_msg.data.num = [float(i)]
            array_msg.data = [float(world_x), float(world_y), 0.1, 0.0, 0.0, 0.0]  # id,x,y,z,roll,pitch,yaw
            
            self.marker_publisher.publish(array_msg)
            self.get_logger().info(f"Published marker_{i} at [{world_x}, {world_y}]")

    # def publish_blue_markers(self, world_result):
    #     for i, result in enumerate(world_result):
    #         world_x, world_y, world_z = result['world_coords']
            
    #         # Create custom message
    #         marker_msg = MarkerData()
            
    #         # Set all the fields from your .msg file
    #         marker_msg.id = i
    #         marker_msg.x = float(world_x)
    #         marker_msg.y = float(world_y)
    #         marker_msg.z = 0.1  # Fixed height or use world_z if available
    #         marker_msg.roll = 0.0
    #         marker_msg.pitch = 0.0
    #         marker_msg.yaw = 0.0
    #         marker_msg.confidence = 1.0  # Optional field
            
    #         # Publish the message
    #         self.marker_publisher.publish(marker_msg)
    #         self.get_logger().info(f"Published marker_{i} at [{world_x}, {world_y}]")

def main():
    """Quick blue detection without GUI"""
    rclpy.init()
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

        node = rclpy.create_node('test_tf')
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
            print(f"  World: X={world_x:.3f}m, Y={world_y:.3f}m, Z={world_z:.3f}m")
            print()
    
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        cv2.waitKey(0)
        publisher.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
