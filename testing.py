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

    def take_screenshot(self, source_type='camera'):
        """Take screenshot from camera or load image file"""
        if source_type == 'camera':
            return self.capture_from_camera()
        else:
            return self.load_image_file()

    def capture_from_camera(self):
        """Capture image from webcam"""
        cap = cv2.VideoCapture(0)

        if not cap.isOpened():
            messagebox.showerror("Error", "Could not access camera")
            return False

        print("Camera opened. Press SPACE to capture, ESC to cancel.")

        while True:
            ret, frame = cap.read()
            if not ret:
                messagebox.showerror("Error", "Failed to capture from camera")
                cap.release()
                return False

            cv2.imshow("Camera - Take Field Screenshot", frame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord(' '):  # Space to capture
                self.image = frame
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"field_screenshot_{timestamp}.jpg"
                cv2.imwrite(filename, frame)
                print(f"Screenshot saved as: {filename}")
                break
            elif key == 27:  # ESC to cancel
                cap.release()
                cv2.destroyAllWindows()
                return False

        cap.release()
        cv2.destroyAllWindows()
        return True

    def load_image_file(self):
        """Load field image from file"""
        root = tk.Tk()
        root.withdraw()

        file_path = filedialog.askopenfilename(
            title="Select Field Image",
            filetypes=[("Image files", "*.jpg *.jpeg *.png *.bmp *.tiff")]
        )

        if file_path and os.path.exists(file_path):
            self.image = cv2.imread(file_path)
            if self.image is not None:
                print(f"Image loaded: {self.image.shape[1]}x{self.image.shape[0]}")
                return True
            else:
                messagebox.showerror("Error", "Could not load image file")
                return False
        return False

    # def detect_blue_areas(self, sensitivity=30):
    #     """Detect blue areas in the image"""
    #     if self.image is None:
    #         return False

    #     hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

    #     # Blue hue range (~120° in HSV)
    #     lower_blue = np.array([120 - sensitivity, 100, 50])
    #     upper_blue = np.array([140 + sensitivity, 255, 255])

    #     blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

    #     # Clean mask
    #     kernel = np.ones((5, 5), np.uint8)
    #     blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)
    #     blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)

    #     contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    #     self.blue_areas = []
    #     for contour in contours:
    #         area = cv2.contourArea(contour)
    #         if area > self.min_blue_area:
    #             x, y, w, h = cv2.boundingRect(contour)
    #             moments = cv2.moments(contour)
    #             if moments["m00"] != 0:
    #                 cx = int(moments["m10"] / moments["m00"])
    #                 cy = int(moments["m01"] / moments["m00"])
    #             else:
    #                 cx, cy = x + w // 2, y + h // 2

    #             self.blue_areas.append({
    #                 'contour': contour,
    #                 'bbox': (x, y, w, h),
    #                 'center': (cx, cy),
    #                 'area': area,
    #                 'pixel_count': cv2.countNonZero(blue_mask[y:y+h, x:x+w])
    #             })

    #     print(f"Detected {len(self.blue_areas)} blue areas")
    #     return blue_mask, self.blue_areas
    
    def detect_blue_areas(self, sensitivity=10, roi=None):
    # """
    # Detect blue areas in the image
    # roi: (x, y, w, h) to limit detection to a specific region of interest
    # """
        if self.image is None:
            return False

        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

        # If ROI is specified, crop the image
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

        summary_text = f"Total Blue Areas: {len(self.blue_areas)}"
        cv2.putText(marked_image, summary_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(marked_image, summary_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 1)

        return marked_image

    def generate_report(self):
        """Generate a detailed report of blue areas"""
        if not self.blue_areas:
            return "No blue areas detected."

        report = f"BLUE AREAS DETECTION REPORT\n"
        report += f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n"
        report += f"Total blue areas found: {len(self.blue_areas)}\n"
        report += f"Minimum area considered: {self.min_blue_area} pixels\n\n"

        total_area = 0
        for i, area in enumerate(self.blue_areas):
            x, y, w, h = area['bbox']
            cx, cy = area['center']

            report += f"BLUE AREA {i+1}:\n"
            report += f"  Position: ({x}, {y})\n"
            report += f"  Size: {w} x {h} pixels\n"
            report += f"  Center: ({cx}, {cy})\n"


        return report

    def save_results(self, marked_image, blue_mask):
        """Save all results to files"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = f"blue_detection_results_{timestamp}"
        os.makedirs(output_dir, exist_ok=True)

        report = self.generate_report()
        with open(f"{output_dir}/detection_report.txt", "w") as f:
            f.write(report)

        areas_data = []
        for i, area in enumerate(self.blue_areas):
            areas_data.append({
                'id': i + 1,
                'bbox': area['bbox'],
                'center': area['center'],
                'area': area['area'],
                'pixel_count': area['pixel_count']
            })

        with open(f"{output_dir}/blue_areas_data.json", "w") as f:
            json.dump(areas_data, f, indent=2)

        print(f"All results saved to: {output_dir}/")
        return output_dir
    
    def perspective_image(self):
        if self.image is None:
            return False

        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        # to do get size of image
        
        #desired points to cut to in image
        pts1 = np.float32([[56,65],[368,52],[28,387],[389,390]])
        
        # size of desired image
        pts2 = np.float32([[0,0],[300,0],[0,300],[300,300]])
        
        M = cv2.getPerspectiveTransform(pts1,pts2)
        
        #image warp size
        dst = cv2.warpPerspective(hsv,M,(300,300))
        cv2.imshow("Perspective Transform", dst)
        return dst


def main():
    detector = BlueAreaDetector()

    print("FIELD BLUE AREA DETECTION SYSTEM")
    print("=" * 40)
    print("1. Take screenshot from camera")
    print("2. Load field image from file")

    try:
        choice = input("Select option (1 or 2): ").strip()

        if choice == "1":
            success = detector.take_screenshot('camera')
        elif choice == "2":
            success = detector.take_screenshot('file')
        else:
            print("Invalid choice")
            return

        if not success:
            return

        print("\nDetecting blue areas...")
        sensitivity = 30
        # Detect only in the middle part of the image
        height, width = detector.image.shape[:2]
        roi = (width//4, height//4, width//2, height//2)
        blue_mask, blue_areas = detector.detect_blue_areas(sensitivity, roi=roi)
        detector.perspective_image()

        #blue_mask, blue_areas = detector.detect_blue_areas(sensitivity)

        if not blue_areas:
            print("No blue areas detected. Try adjusting sensitivity.")
            return

        marked_image = detector.mark_blue_areas(blue_mask)

        cv2.imshow("Original Field", detector.image)
        cv2.imshow("Blue Areas Marked", marked_image)
        cv2.imshow("Blue Mask", blue_mask)

        print("\n" + "=" * 50)
        print(detector.generate_report())
        print("=" * 50)

        print("\nPress any key on space to continue...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        save = input("\nSave results? (y/n): ").strip().lower()
        if save == 'y':
            output_dir = detector.save_results(marked_image, blue_mask)
            print(f"Results saved to: {output_dir}")

    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        cv2.destroyAllWindows()


def quick_blue_detection():
    """Quick blue detection without GUI"""
    detector = BlueAreaDetector()

    if detector.load_image_file():
        blue_mask, blue_areas = detector.detect_blue_areas()
        marked_image = detector.mark_blue_areas(blue_mask)

        cv2.imshow("Blue Areas Detected", marked_image)
        cv2.imshow("Blue Mask", blue_mask)

        print(detector.generate_report())

        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
