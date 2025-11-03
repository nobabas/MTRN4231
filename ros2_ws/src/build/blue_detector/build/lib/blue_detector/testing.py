import cv2
import numpy as np
import tkinter as tk
from tkinter import filedialog, messagebox
import os
from datetime import datetime
import json

class BlueAreaDetector:
    def __init__(self):
        self.image = None
        self.blue_areas = []
        self.min_blue_area = 100  # Minimum area to consider as a blue region (pixels)

    def load_image_file(self):
        """Load field image from file"""
        root = tk.Tk()
        root.withdraw()

        #file_path = "//home/mtrn/4231/received_images/received_current.png"
        file_path = "/home/mtrn/4231/received_images/current_image.jpg"
        if file_path:
            self.image = cv2.imread(file_path)
            if self.image is not None:
                print(f"Image loaded: {self.image.shape[1]}x{self.image.shape[0]}")
                return True
            else:
                messagebox.showerror("Error", "Could not load image file")
                return False
        return False
    
    def detect_blue_areas(self, sensitivity=10, roi=None):
        if self.image is None:
            return False

        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

        
        if roi:
            x, y, w, h = roi
            hsv = hsv[y:y+h, x:x+w]

        # Define blue range
        lower_blue = np.array([120 - sensitivity, 100, 50])
        upper_blue = np.array([120 + sensitivity, 255, 255])

        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Clean mask
        kernel = np.ones((5, 5), np.uint8)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        self.blue_areas = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.min_blue_area:
                x_c, y_c, w_c, h_c = cv2.boundingRect(contour)

                # Adjust positions if cropped
                if roi:
                    x_c += x
                    y_c += y

                moments = cv2.moments(contour)
                if moments["m00"] != 0:
                    cx = int(moments["m10"] / moments["m00"]) + (x if roi else 0)
                    cy = int(moments["m01"] / moments["m00"]) + (y if roi else 0)
                else:
                    cx, cy = x_c + w_c // 2, y_c + h_c // 2

                self.blue_areas.append({
                    'contour': contour,
                    'bbox': (x_c, y_c, w_c, h_c),
                    'center': (cx, cy),
                    'area': area,
                    'pixel_count': cv2.countNonZero(blue_mask[y_c - (y if roi else 0):y_c - (y if roi else 0) + h_c, x_c - (x if roi else 0):x_c - (x if roi else 0) + w_c])
                })

        print(f"Detected {len(self.blue_areas)} blue areas in ROI")
        return blue_mask, self.blue_areas


    def mark_blue_areas(self, blue_mask=None, display_original=True):
        """Mark blue areas on the image with bounding boxes and labels"""
        if self.image is None:
            return None

        if display_original:
            marked_image = self.image.copy()
        else:
            marked_image = np.zeros_like(self.image)

        for i, area in enumerate(self.blue_areas):
            x, y, w, h = area['bbox']
            cv2.rectangle(marked_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.drawContours(marked_image, [area['contour']], -1, (0, 255, 255), 2)

            cx, cy = area['center']
            cv2.circle(marked_image, (cx, cy), 5, (0, 0, 255), -1)

            label = f"Blue {i+1}: {area['area']:.0f}px"
            cv2.putText(marked_image, label, (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(marked_image, label, (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 1)

        return marked_image

    def generate_report(self):
        """Generate a detailed report of blue areas"""
        if not self.blue_areas:
            return "No blue areas detected."

        report = f"BLUE AREAS DETECTION REPORT\n"
        report += f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n"
        report += f"Total blue areas found: {len(self.blue_areas)}\n"
        
        self.xcoord = []
        self.ycoord = []
        
        for i, area in enumerate(self.blue_areas):
            x, y, w, h = area['bbox']
            cx, cy = area['center']

            report += f"BLUE AREA {i+1}:\n"
            report += f"{cx}, {cy}, 0.1, 0, 0, 0\n"
            
            self.xcoord.append(cx)
            self.ycoord.append(cy)

        return report
    
    # In your BlueAreaDetector.get_world_coordinates() method:
    def get_world_coordinates(self, tf_handler, depth_value):
        world_coordinates = []
        
        for area in self.blue_areas:
            cx, cy = area['center']
            
            # SINGLE FUNCTION CALL - that's all you need!
            world_coords = tf_handler.pixel_to_3d(cx, cy, depth_value)
            
            world_coordinates.append({
                'pixel_center': (cx, cy),
                'world_coords': world_coords,
                'area_data': area
            })
        
        return world_coordinates
    
def main():
    """Quick blue detection without GUI"""
    detector = BlueAreaDetector()

    if detector.load_image_file():
        detector.image = cv2.cvtColor(detector.image, cv2.COLOR_BGR2RGB)
        blue_mask, _ = detector.detect_blue_areas()
        marked_image = detector.mark_blue_areas(blue_mask)
        
        cv2.imshow("Blue Areas Detected", marked_image)
        cv2.imshow("Blue Mask", blue_mask)

        print(detector.generate_report())
    
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
