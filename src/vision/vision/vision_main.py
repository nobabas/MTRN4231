import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Polygon, Point32  # We'll use Polygon to send a list of points
from ultralytics import YOLO
import time
from interface.msg import MarkerData

# --- Your Model and Image Paths ---
# This will need to be changed
MODEL_PATH = '/home/mtrn/src/best.pt'
IMAGE_PATH = "/home/mtrn/4231/received_images/current_image.jpg"
#####################################

# Detection confidence threshold
CONFIDENCE = 0.1

# ------------------------------------

class YoloPublisher(Node):
    def __init__(self):
        super().__init__('yolo_publisher')
        

        # Load the YOLO model
        self.get_logger().info(f'Loading model from {MODEL_PATH}...')
        try:
            self.model = YOLO(MODEL_PATH)
            self.get_logger().info('Model loaded successfully.')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            return
        
        self.marker_publisher = self.create_publisher(
            MarkerData,  # Changed this
            '/pixel_coords', 
            10
        )
        
    def run_detection_and_publish(self):
        self.get_logger().info(f'Running detection on {IMAGE_PATH}...')
        results = self.model(IMAGE_PATH, conf=CONFIDENCE)

        if results:
            r = results[0]  # Get the first result
            boxes = r.boxes.xyxy  # [x1, y1, x2, y2]
            classes = r.boxes.cls
            self.get_logger().info(f'Found {len(boxes)} detections.')

            for i in range(len(boxes)):
                box = boxes[i]
                
                
                # Your original calculation
                x1, y1, x2, y2 = box.tolist()
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2

                marker_msg = MarkerData()
            
                marker_msg.id = float(i)
                marker_msg.pose = [float(cx), float(cy)]

                self.marker_publisher.publish(marker_msg)
                
                self.get_logger().info(f'Published marker: id={marker_msg.id}, pose={marker_msg.pose}')

def main(args=None):
    rclpy.init(args=args)
    
    yolo_publisher_node = YoloPublisher()
    
    # Run the detection and publish
    yolo_publisher_node.run_detection_and_publish()
    
    # Give a moment for the message to be sent before shutting down
    time.sleep(100.0) 
    
    # Clean up
    yolo_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()