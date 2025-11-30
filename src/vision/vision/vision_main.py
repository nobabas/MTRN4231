import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from interfaces.msg import MarkerData
from ultralytics import YOLO
import cv2
import numpy as np

# Make sure to change
MODEL_PATH = '/home/samuel/MTRN4231/src/best.pt'

CONFIDENCE = 0.25

class YoloPublisher(Node):
    def __init__(self):
        super().__init__('yolo_publisher')

        # Load YOLO
        self.get_logger().info(f"Loading model from {MODEL_PATH}")
        self.model = YOLO(MODEL_PATH)

        self.frame_count = 0
        self.process_rate = 6

        # Subscribe to camera stream
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )

        # Publish marker outputs
        self.marker_pub = self.create_publisher(MarkerData, '/pixel_coords', 10)

        self.get_logger().info("YOLO live detection node started.")

    def image_callback(self, msg: Image):
        self.frame_count += 1
        if self.frame_count % self.process_rate != 0:
            return

        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # Run YOLO on frame
        results = self.model(frame, conf=CONFIDENCE, verbose=False)
        r = results[0]

        # Draw annotated detections
        annotated = r.plot()
        # Show the live YOLO feed
        cv2.imshow("YOLO Live Annotated", annotated)
        cv2.waitKey(1)

        # Publish centroids for every detection
        boxes = r.boxes.xyxy
        for i, box in enumerate(boxes):
            x1, y1, x2, y2 = box.tolist()
            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2

            msg_out = MarkerData()
            msg_out.id = float(i)
            msg_out.pose = [float(cx), float(cy)]
            self.marker_pub.publish(msg_out)

            # self.get_logger().info(f"Published marker: {cx}, {cy}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()