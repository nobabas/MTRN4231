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

class TFHandler:
    """Handles TF transforms and camera intrinsics for blue marker detection."""

    def __init__(self, node):
        self.node = node
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
        self.broadcaster = tf2_ros.TransformBroadcaster(self.node)
        self.intrinsics = None

        # Subscribe to camera info
        self.cam_info_sub = self.node.create_subscription(
            CameraInfo,
            #Based on parameters
            '/camera/camera/aligned_depth_to_color/camera_info',
            self.camera_info_callback,
            10,
        )

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
        
    # def transform_camera_to_world(self, point):
    #     """Custom transformation from camera to world coordinates."""
    #     return [
            
    #         -point[0] + 0.48802, #+ 0.57,   # this is -(robot x)
    #         point[1] + 0.02131, #- 0.691,  # this is -(robot y)
    #         point[2],   # Camera Z -> World X (forward)
    #     ]
    
    def transform_camera_to_world(self, point_3d, from_frame='camera_link', to_frame='base_link'):
        """
        Transform a 3D point given in camera frame to the world/base frame.
        point_3d: tuple or list (x, y, z) in the camera frame
        from_frame: source frame id (default 'camera_link')S
        to_frame: target frame id (default 'base_link')
        returns: (x, y, z) in target frame
        """
        ps = PointStamped()
        ps.header.frame_id = from_frame
        # use zero time to get latest transform
        ps.header.stamp = rclpy.time.Time().to_msg()
        ps.point.x, ps.point.y, ps.point.z = point_3d

        try:
            # assumes self.tf_buffer is an instance of tf2_ros.Buffer
            transform = self.tf_buffer.lookup_transform(
                to_frame,
                from_frame,
                rclpy.time.Time())  # latest
            
            transformed = tf2_geometry.do_transform_point(ps, transform)
            return transformed.point
        except Exception as e:
            raise RuntimeError(f"Failed to transform point from {from_frame} to {to_frame}: {e}")

    def pixel_to_3d(self, pixel_x, pixel_y, depth_value):
        """Convert pixel+depth to 3D point in camera frame"""            
        # Convert to camera frame coordinates (X right, Y down, Z forward)
        point_3d = rs.rs2_deproject_pixel_to_point(
            self.intrinsics,
            [pixel_x,pixel_y],
            depth_value * 0.001  # mm to meters
        )
        world_point = self.camera_point_to_world(point_3d)
        if world_point is None:
            return None
        return [world_point.x, world_point.y, world_point.z]

    