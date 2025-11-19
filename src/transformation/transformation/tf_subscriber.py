from rclpy.node import Node
from interfaces.msg import MarkerData
import threading
import time

TOPIC_NAME = '/pixel_coords'

class MarkerSubscriber(Node):
    def __init__(self):
        # 1. Initialize the node
        super().__init__('marker_subscriber')
        self.blue_area = []
        self.data_received = False
        self.data_received_event = threading.Event()
        
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
        
        # Convert float32 array to regular Python list for proper handling
        pose_list = list(marker_pose)
        # Only add a new marker if we have less than 4
        if len(self.blue_area) < 4:
            self.blue_area.append({
                'center': pose_list
            })
            # 3. Log the received data
            self.get_logger().info(f'Received marker: ID={marker_id:.1f}, Pose={pose_list}')
        else:
            self.get_logger().info("Maximum of 4 markers already collected, ignoring extra markers.")
        
        # Set flags to indicate data was received
        self.data_received = True
        self.data_received_event.set()

    def wait_for_data(self, timeout=10.0):
        self.get_logger().info(f'Waiting for marker data (timeout: {timeout} seconds)...')
        
        # If data was already received, return immediately
        if self.data_received:
            self.get_logger().info('Data already received!')
            return True
            
        # Wait for the event with timeout
        received = self.data_received_event.wait(timeout=timeout)
        
        if received:
            self.get_logger().info(f'Successfully received {len(self.blue_area)} marker(s)!')
        else:
            self.get_logger().warning(f'No data received within {timeout} seconds')
            
        return received

    def get_world_coordinates(self, tf_handler, depth_value):
        markerLimit = 4
        if not self.blue_area:
            self.get_logger().warning('No marker data available for conversion')
            return []
        markerdata = self.blue_area[:markerLimit]
        world_coordinates = []
        
        for i, area in enumerate(markerdata):
            cx, cy = area['center']
            
            # SINGLE FUNCTION CALL - that's all you need!
            world_coords = tf_handler.pixel_to_3d(cx, cy, depth_value)
            
            world_coordinates.append({
                'id': i,
                'pixel_center': (cx, cy),
                'world_coords': world_coords,
            })
        
        return world_coordinates

    def reset(self):
        """
        Reset the subscriber to wait for new data
        """
        self.blue_area.clear()
        self.data_received = False
        self.data_received_event.clear()
        self.get_logger().info('Subscriber reset - ready for new data')