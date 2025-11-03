#!THIS CODE WILL NOT USED JUST TO UNDERSTAND THE FUNCTIONS

import rclpy
from tf_utils import TFHandler
from geometry_msgs.msg import Point
from ros2commands import run_static_transform, stop_static_transform
def main():
    rclpy.init()
    node = rclpy.create_node('test_tf')
    tf_handler = TFHandler(node)
    process = run_static_transform()
    
    print("Testing TF transformations...")
    
    # Wait a bit for TF system
    rclpy.spin_once(node, timeout_sec=2.0)
    
    # Test 1: Transform a 3D point from camera frame to base frame
    print("\n1. Testing 3D point transformation:")
    point_msg = Point()
    point_msg.x = 0.0    # meters in camera frame (right)
    point_msg.y = 0.0    # meters in camera frame (down) 
    point_msg.z = 1.0    # meters in camera frame (forward)
    
    base_point = tf_handler.transform_to_base(point_msg, 'camera_link')

    #ros2 run tf2_ros static_transform_publisher 1.30317 0.0174152 0.675776 -0.388123 -0.0054127 0.92155 0.0087602 camera_link base

    if base_point:
        print(f"   Camera point (0,0,1) -> Base: ({base_point.x:.3f}, {base_point.y:.3f}, {base_point.z:.3f})")
    
    # Test 2: Transform pixel coordinates to 3D world coordinates
    print("\n2. Testing pixel to 3D transformation:")
    # In your test, after creating tf_handler:
    if tf_handler.intrinsics is None:
        tf_handler.set_mock_intrinsics()
        
    world_coords = tf_handler.pixel_to_3d(96, 295, 1000)  # center pixel, 1m depth
    print(f"   Pixel (320,240) depth 1000mm -> World: {world_coords}")
    
    # Test 3: Direct camera to world coordinate transform
    print("\n3. Testing direct coordinate transform:")
    camera_coords = [0.1, 0.2, 1.5]  # x, y, z in camera frame
    world_coords = tf_handler.transform_camera_to_world(camera_coords)
    print(f"   Camera {camera_coords} -> World {world_coords}")
    
    stop_static_transform(process)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()