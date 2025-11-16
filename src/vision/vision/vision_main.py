import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Polygon, Point32  # We'll use Polygon to send a list of points
from ultralytics import YOLO
import time
from interfaces.msg import MarkerData
import cv2
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

# --- Your Model and Image Paths ---
# This will need to be changed
MODEL_PATH = '/home/samuel/MTRN4231/src/best.pt'
IMAGE_PATH = '/home/samuel/MTRN4231/yolo_dataset/train/images/image1.jpg'
CONFIDENCE = 0.5
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
        
        # --- 2. DEFINE STICKY QOS ---
        transient_local_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # --- 3. UPDATE PUBLISHER ---
        self.marker_publisher = self.create_publisher(
            MarkerData,
            '/pixel_coords', 
            qos_profile=transient_local_qos
        )

    def run_detection_and_publish(self):
        self.get_logger().info(f'Running detection on {IMAGE_PATH}...')
        results = self.model(IMAGE_PATH, conf=CONFIDENCE)

        if results:
            r = results[0]
            boxes = r.boxes.xyxy
            classes = r.boxes.cls
            self.get_logger().info(f'Found {len(boxes)} detections.')

            for i in range(len(boxes)):
                box = boxes[i]
                x1, y1, x2, y2 = box.tolist()
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2

                marker_msg = MarkerData()
                marker_msg.id = float(i)
                marker_msg.pose = [float(cx), float(cy)]

                self.marker_publisher.publish(marker_msg)
                self.get_logger().info(f'Published marker: id={marker_msg.id}, pose={marker_msg.pose}')

    def image(self):
        results = self.model(IMAGE_PATH, conf=CONFIDENCE)

        # --- SAVE ANNOTATED IMAGE ---
        annotated = results[0].plot()  # Draw boxes, labels, masks, etc.
        scale = 0.1  # change to 0.3 for smaller, 0.7 for larger
        new_width = int(annotated.shape[1] * scale)  
        new_height = int(annotated.shape[0] * scale)
        resized_image = cv2.resize(annotated, (new_width, new_height))
        cv2.imshow('Annotated Image', resized_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()



def main(args=None):
    rclpy.init(args=args)
    
    yolo_publisher_node = YoloPublisher()
    
    # Run the detection and publish
    yolo_publisher_node.run_detection_and_publish()
    
    yolo_publisher_node.image()

    # Give a moment for the message to be sent before shutting down
    time.sleep(10.0)
    
    # Clean up
    yolo_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()