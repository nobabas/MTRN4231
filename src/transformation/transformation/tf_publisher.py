from interface.msg import MarkerData
from rclpy.node import Node
import rclpy

class BlueMarkerPublisher(Node):
    def __init__(self):
        super().__init__('blue_marker_publisher')
        
        self.marker_publisher = self.create_publisher(
            MarkerData,
            '/blue_markers_coords', 
            10
        )
        
        self.create_timer(2.0, self.check_publisher_status)
        self.get_logger().info("BlueMarkerPublisher initialized with MarkerData message")
    
    def check_publisher_status(self):
        subscription_count = self.marker_publisher.get_subscription_count()
        self.get_logger().info(f"Publisher has {subscription_count} subscribers")
    
    def publish_blue_markers(self, world_result):
        if not world_result:
            self.get_logger().warning("No world results to publish")
            return
            
        self.get_logger().info(f"Publishing {len(world_result)} markers")
        
        for i, result in enumerate(world_result):
            world_x, world_y, world_z = result['world_coords']
            
            # Create custom message
            marker_msg = MarkerData()
            
            # Set the fields according to your interface
            marker_msg.id = float(i)  # float32
            # pose is float32[] - make sure it's exactly 6 elements for position + orientation
            marker_msg.pose = [float(world_x), float(world_y), float(world_z), 0.0, 0.0, 0.0]
            
            # Publish the message
            self.marker_publisher.publish(marker_msg)
            self.get_logger().info(f"Published marker_{i} at [{world_x:.2f}, {world_y:.2f}, {world_z:.2f}]")