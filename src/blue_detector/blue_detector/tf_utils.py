# tf_utils.py
import tf_transformations  
import rclpy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Point, Pose, Quaternion, TransformStamped
from rclpy.time import Time
import pyrealsense2 as rs
from sensor_msgs.msg import CameraInfo


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
        """Transform a Point from the given frame to the 'base' frame."""
        try:
            if self.tf_buffer.can_transform('base', from_frame, Time(), timeout=rclpy.duration.Duration(seconds=2.0)):
                transform = self.tf_buffer.lookup_transform(
                    'base', from_frame, Time(),
                    timeout=rclpy.duration.Duration(seconds=2.0)
                )

                pose = Pose()
                pose.position = point
                pose.orientation = Quaternion()  # identity orientation

                transformed = tf2_geometry_msgs.do_transform_pose(pose, transform)
                return transformed.position
            else:
                self.node.get_logger().warn(f'No transform from {from_frame} → base found.')
                return None
        except Exception as e:
            self.node.get_logger().error(f"TF error: {e}")
            return None

    def transform_point(self, point, target_frame, source_frame):
        """General-purpose transform: convert a Point from source_frame → target_frame."""
        try:
            if self.tf_buffer.can_transform(target_frame, source_frame, Time(), timeout=rclpy.duration.Duration(seconds=2.0)):
                transform = self.tf_buffer.lookup_transform(
                    target_frame, source_frame, Time(),
                    timeout=rclpy.duration.Duration(seconds=2.0)
                )

                pose = Pose()
                pose.position = point
                pose.orientation = Quaternion()  # identity

                transformed = tf2_geometry_msgs.do_transform_pose(pose, transform)
                return transformed.position
            else:
                self.node.get_logger().warn(f'No transform from {source_frame} → {target_frame}.')
                return None
        except Exception as e:
            self.node.get_logger().error(f"TF error: {e}")
            return None

    def transform_camera_to_world(self, point):
        """Custom transformation from camera to world coordinates."""
        return [
            point[2],   # Camera Z → World X
            -point[1] + 0.038 + 0.20,   # Adjusted Y offset
            -point[0] - 0.18,           # Adjusted Z offset
        ]

    def pixel_to_3d(self, pixel_x, pixel_y, depth_value):
        """Convert pixel and depth to 3D point in camera frame."""
        if self.intrinsics is None:
            self.node.get_logger().warn("No intrinsics set — using mock values.")
            self.set_mock_intrinsics()

        point_3d = rs.rs2_deproject_pixel_to_point(
            self.intrinsics,
            [pixel_y, pixel_x],
            depth_value * 0.001  # mm → meters
        )
        return self.transform_camera_to_world(point_3d)
