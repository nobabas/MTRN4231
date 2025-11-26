import rclpy
import tf2_ros
from geometry_msgs.msg import Point, Pose, TransformStamped, PointStamped
from rclpy.time import Time
import pyrealsense2 as rs
from sensor_msgs.msg import CameraInfo
from rclpy.callback_groups import ReentrantCallbackGroup
import tf2_geometry_msgs
import tf2_geometry_msgs.tf2_geometry_msgs as tf2_geometry
from geometry_msgs.msg import Quaternion
from tf2_geometry_msgs import do_transform_pose

import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class TFHandler:
    """Handles TF transforms and camera intrinsics for blue marker detection."""

    def __init__(self, node):
        self.node = node
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
        self.broadcaster = tf2_ros.TransformBroadcaster(self.node)
        self.intrinsics = None
        self.bridge = CvBridge()
        self.current_depth = None

        # Subscribe to camera info
        self.cam_info_sub = self.node.create_subscription(
            CameraInfo,
            #Based on parameters
            '/camera/camera/aligned_depth_to_color/camera_info',
            self.camera_info_callback,
            10,
        )

        self.depth_sub = self.node.create_subscription(
            Image,
            #Based on parameters
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10,
        )

    def depth_callback(self, msg):
        self.current_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def camera_info_callback(self, msg):
        """Store camera intrinsics when available."""
        if self.intrinsics is None:
            self.intrinsics = rs.intrinsics()
            self.intrinsics.width = msg.width
            self.intrinsics.height = msg.height
            self.intrinsics.ppx = msg.k[2]
            self.intrinsics.ppy = msg.k[5]
            self.intrinsics.fx = msg.k[0]
            self.intrinsics.fy = msg.k[4]
            self.intrinsics.model = rs.distortion.brown_conrady
            self.intrinsics.coeffs = list(msg.d)

            # Print out the camera specifications
            print("=== CAMERA INTRINSICS SPECIFICATIONS ===")
            print(f"Image Resolution: {self.intrinsics.width} x {self.intrinsics.height}")
            print(f"Principal Point (optical center): ({self.intrinsics.ppx:.2f}, {self.intrinsics.ppy:.2f})")
            print(f"Focal Length: fx={self.intrinsics.fx:.2f}, fy={self.intrinsics.fy:.2f}")
            print(f"Distortion Model: {self.intrinsics.model}")
            print(f"Distortion Coefficients: {self.intrinsics.coeffs}")
            print("========================================")

            self.node.get_logger().info("Camera intrinsics received")


    def set_mock_intrinsics(self):
        """Set mock camera intrinsics for testing when a real camera isn't available."""
        self.intrinsics = rs.intrinsics()
        self.intrinsics.width = 640
        self.intrinsics.height = 480
        self.intrinsics.ppx = 320.0
        self.intrinsics.ppy = 240.0
        self.intrinsics.fx = 525.0
        self.intrinsics.fy = 525.0
        self.intrinsics.model = rs.distortion.brown_conrady
        self.intrinsics.coeffs = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.node.get_logger().info("Mock camera intrinsics set for testing")

    def transform_to_base(self, point, from_frame='camera_link'):
        try:
            if self.tf_buffer.can_transform('base', from_frame, Time(), timeout=rclpy.duration.Duration(seconds=2.0)):
                transform = self.tf_buffer.lookup_transform(
                    'base', from_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=2.0))
                
                pose = Pose()
                pose.position = point   
                pose.orientation = Quaternion()  # Add this line to prevent NoneType errors
                
                transformed = tf2_geometry_msgs.do_transform_pose(pose, transform)
                return transformed.position
            else:
                self.node.get_logger().error('TF transform failed: could not find transform from %s to base' % from_frame)
                return None
        except Exception as e:
            self.node.get_logger().error(f"TF error: {str(e)}")
            return None
            
    def publish_transform(self, frame_id, child_frame_id, point, orientation=None):
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = frame_id
        t.child_frame_id = child_frame_id
        t.transform.translation.x = point[0]
        t.transform.translation.y = point[1]
        t.transform.translation.z = point[2]
        
        # Update from Sam: Removed need for tf_transformation
        #             q = tf_transformations.quaternion_from_euler(0, 0, 0)
        if orientation is None:
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
        else:
            t.transform.rotation = orientation
            
        self.broadcaster.sendTransform(t)
        
    def transform_camera_to_world(self, point):
        """Custom transformation from camera to world coordinates."""
        return [
            
            point[0] + 0.81,   # this is -(robot x)
            point[1] - 0.09, # this is -(robot y)
            point[2],   # Camera Z -> World X (forward)
        ]
    
    def pixel_to_3d(self, pixel_x, pixel_y):
        """Convert pixel+depth to 3D point in camera frame"""            
        # Convert to camera frame coordinates (X right, Y down, Z forward)
        # Ensure depth image exists
        if self.current_depth is None:
            self.node.get_logger().warn("Depth frame not yet received")
            return None

        # Get depth at pixel
        d = float(self.current_depth[int(pixel_y), int(pixel_x)]) * 0.001  # mm → meters

        # Handle invalid/missing depth
        if d <= 0 or np.isnan(d):
            self.node.get_logger().warn(f"Invalid depth at pixel ({pixel_x}, {pixel_y})")
            return None

        point_3d = rs.rs2_deproject_pixel_to_point(
            self.intrinsics,
            [float(pixel_y), float(pixel_x)],
            d
        )

        return self.transform_camera_to_world(point_3d)