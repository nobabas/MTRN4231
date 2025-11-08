from rclpy.node import Node
from interface.msg import MarkerData  # Your custom message

TOPIC_NAME = '/pixel_coords'

class MarkerSubscriber(Node):
    def __init__(self):
        # 1. Initialize the node
        super().__init__('marker_subscriber')
        self.blue_area = []
        # 2. Create the subscriber
        self.subscription = self.create_subscription(
            MarkerData,
            TOPIC_NAME,
            self.listener_callback,  # Function to run when a message is received
            10)
        
        self.get_logger().info(f'Subscribing to "{TOPIC_NAME}". Waiting for messages...')

    def listener_callback(self, msg):
        """
        This function is called every time a message is received.
        """
        # The 'msg' object is an instance of MarkerData
        marker_id = msg.id
        marker_pose = msg.pose
        
        self.blue_area.append({
            'center': marker_pose
        })

        # 3. Log the received data to the console
        self.get_logger().info(f'Received marker: ID={marker_id:.1f}, Pose={marker_pose}')

    def get_world_coordinates(self, tf_handler, depth_value):
        world_coordinates = []
        
        for area in self.blue_area:
            cx, cy = area['center']
            
            # SINGLE FUNCTION CALL - that's all you need!
            world_coords = tf_handler.pixel_to_3d(cx, cy, depth_value)
            
            world_coordinates.append({
                'pixel_center': (cx, cy),
                'world_coords': world_coords,
            })
        
        return world_coordinates