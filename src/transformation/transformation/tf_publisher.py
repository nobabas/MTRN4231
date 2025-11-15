from interfaces.msg import Marker2D, Marker2DArray
from rclpy.node import Node

class BlueMarkerPublisher(Node):
    def __init__(self):
        super().__init__('blue_marker_publisher')
        
        self.marker_publisher = self.create_publisher(
            Marker2DArray,
            '/blue_markers_coords', 
            10
        )
        
        self.create_timer(2.0, self.check_publisher_status)
        self.get_logger().info("BlueMarkerPublisher initialized with MarkerData message")
    
    def check_publisher_status(self):
        subscription_count = self.marker_publisher.get_subscription_count()
        self.get_logger().info(f"Publisher has {subscription_count} subscribers")
    
    def publish_blue_markers(self, world_result):
        # Create marker array message
        marker_array_msg = MarkerArray2D()
        
        for i, result in enumerate(world_result):
            world_x, world_y, world_z = result['world_coords']
            
            # Create individual marker message
            marker_msg = Marker2D()
            marker_msg.array = [float(i), float(world_x), float(world_y)]
            
            marker_array_msg.markers.append(marker_msg)
            
            self.get_logger().info(
                f"Marker_{i}, x={world_x:.2f}, y={world_y:.2f}"
            )

        # Publish the structured message
        self.marker_publisher.publish(marker_array_msg)
        
        self.get_logger().info(
            f"Published {len(world_result)} markers as structured data."
        )