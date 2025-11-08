import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np

class Vision(Node):
    def __init__(self):
        super().__init__('vision')
        self.bridge = CvBridge()
        
        # Load YOLO model
        self.model = YOLO('/home/mtrn/lab4-main/runs/detect/train9/weights/best.pt')
        self.model.verbose = False
        
        # Performance optimization settings
        self.subscription = self.create_subscription(
            Image, 
            '/image_raw', 
            self.image_callback, 
            rclpy.qos.qos_profile_sensor_data  # Use sensor data QoS for real-time
        )
        
        # Performance tracking
        self.frame_count = 0
        self.last_time = self.get_clock().now()
        
        # Optimization flags
        self.display_enabled = True  # Set to False for headless operation
        
        self.get_logger().info("Vision node initialized with real-time optimizations")

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Run YOLO inference with optimizations
            results = self.model(
                cv_image, 
                verbose=False,
                imgsz=640,  # Fixed size for consistent performance
                half=False,  # Set to True if using GPU with FP16 support
                conf=0.5,   # Confidence threshold to reduce processing
                max_det=10  # Limit maximum detections
            )

            # Process results
            if results and len(results) > 0:
                annotated_image = self.process_results(results[0], cv_image)
                
                # Display results
                if self.display_enabled:
                    cv2.imshow('Real-time Segmentation', annotated_image)
                    cv2.waitKey(1)
            
            # Performance monitoring (optional)
            self.frame_count += 1
            if self.frame_count % 30 == 0:  # Log every 30 frames
                current_time = self.get_clock().now()
                elapsed = (current_time - self.last_time).nanoseconds / 1e9
                fps = 30 / elapsed if elapsed > 0 else 0
                self.get_logger().info(f"Processing at {fps:.1f} FPS")
                self.last_time = current_time
                
        except Exception as e:
            self.get_logger().error(f"Error in image callback: {str(e)}")

    def process_results(self, result, original_image):
        """Process YOLO results and annotate image"""
        # Use YOLO's built-in plotting (optimized)
        annotated_image = result.plot()
        
        # Process masks for person detection
        if result.masks is not None:
            for i, mask in enumerate(result.masks):
                # Check if detection is a person
                if int(result.boxes[i].cls[0]) == 0:  # Assuming class 0 is 'person'
                    binary_mask = mask.data.cpu().numpy().squeeze()
                    
                    # Find contours
                    contours, _ = cv2.findContours(
                        binary_mask.astype(np.uint8), 
                        cv2.RETR_EXTERNAL, 
                        cv2.CHAIN_APPROX_SIMPLE
                    )
                    
                    if contours:
                        largest_contour = max(contours, key=cv2.contourArea)
                        M = cv2.moments(largest_contour)
                        if M["m00"] != 0:
                            centroid_x = int(M["m10"] / M["m00"])
                            centroid_y = int(M["m01"] / M["m00"])
                            
                            # Draw centroid
                            cv2.circle(annotated_image, (centroid_x, centroid_y), 5, (0, 0, 255), -1)
                            
                            # Add coordinates text
                            self.draw_centroid_text(annotated_image, centroid_x, centroid_y)
        
        return annotated_image

    def draw_centroid_text(self, image, x, y):
        """Optimized text drawing for centroid coordinates"""
        text = f"({x}, {y})"
        text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
        text_x = x - text_size[0] // 2
        text_y = y + 20
        
        # Draw background rectangle
        cv2.rectangle(
            image, 
            (text_x, text_y - text_size[1] - 2), 
            (text_x + text_size[0], text_y + 2), 
            (0, 0, 0), 
            -1
        )
        
        # Draw text
        cv2.putText(
            image, text, (text_x, text_y), 
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1
        )

    def destroy_node(self):
        """Cleanup before destruction"""
        if self.display_enabled:
            cv2.destroyAllWindows()
        super().destroy_node()

def main():
    rclpy.init()
    
    try:
        vision_node = Vision()
        rclpy.spin(vision_node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'vision_node' in locals():
            vision_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()