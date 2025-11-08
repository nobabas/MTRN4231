from interface.msg import MarkerData

from rclpy.node import Node

class BlueMarkerPublisher(Node):
    def __init__(self):
        super().__init__('blue_marker_publisher')
        
        # Publish as Float32MultiArray messages
        self.marker_publisher = self.create_publisher(
            MarkerData,  # Changed this
            '/blue_markers_coords', 
            10
        )
    
    def publish_blue_markers(self, world_result):
        for i, result in enumerate(world_result):
            world_x, world_y, world_z = result['world_coords']
            
            # Create custom message
            marker_msg = MarkerData()
            
            # Set all the fields from your .msg file
            marker_msg.id = float(i)
            marker_msg.pose = [float(world_x), float(world_y), 0.1, 0.0, 0.0, 0.0]
            
            # Publish the message
            self.marker_publisher.publish(marker_msg)
            self.get_logger().info(f"Published marker_{i} at [{world_x}, {world_y}]")
