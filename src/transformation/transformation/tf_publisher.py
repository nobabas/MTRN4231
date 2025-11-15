import rclpy
from rclpy.node import Node
from interfaces.msg import Marker2DArray
# --- 1. ADD QOS IMPORTS ---
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

class BlueMarkerPublisher(Node):
    def __init__(self):
        super().__init__('blue_marker_publisher')
        
        # --- 2. DEFINE STICKY QOS ---
        transient_local_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # --- 3. CREATE PUBLISHER WITH QOS ---
        self.marker_publisher = self.create_publisher(
            Marker2DArray,
            '/blue_markers_coords', 
            qos_profile=transient_local_qos # <-- Use the sticky profile
        )

    def publish_blue_markers(self, world_result):
        # If no markers found, just return (or publish empty array)
        if not world_result:
            return

        msg = Marker2DArray()
        
        for result in world_result:
            # We assume 'result' has keys: 'id', 'pixel_center', 'world_coords'
            # (This matches what we set up in tf_subscriber.py)
            
            # Create a temporary object to hold data
            # Note: Marker2DArray.msg holds a list of Marker2D
            # Marker2D.msg has fields: float32 id, float32 x, float32 y
            
            from interfaces.msg import Marker2D
            marker = Marker2D()
            
            marker.id = float(result['id'])
            # We send the WORLD coordinates here, not pixel coordinates
            marker.x = float(result['world_coords'][0]) 
            marker.y = float(result['world_coords'][1])
            
            msg.markers.append(marker)

        self.marker_publisher.publish(msg)
        self.get_logger().info(f'Published {len(msg.markers)} markers to /blue_markers_coords')