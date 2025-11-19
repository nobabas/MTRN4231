import cv2
import numpy as np
from ultralytics import YOLO

# Load your trained model
# Copy path file
model = YOLO('/home/mtrn/MTRN4231_soil/src/best.pt')

# Load a local image file
# Copy path file of desired image in dataset
image_path = '/home/mtrn/Pictures/Screenshots/Picture.png'
cv_image = cv2.imread(image_path)

# Run YOLO detection
results = model(cv_image, conf=0.5)

# Draw detections
annotated_image = results[0].plot()

# Extract bounding boxes and calculate centers
for box in results[0].boxes.xyxy:
    x1, y1, x2, y2 = map(int, box)
    cx = int((x1 + x2) / 2)
    cy = int((y1 + y2) / 2)

    # Draw center point and coordinates
    cv2.circle(annotated_image, (cx, cy), 5, (0, 0, 255), -1)
    text = f"({cx}, {cy})"
    text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
    text_x = cx - text_size[0] // 2
    text_y = cy + 20
    cv2.rectangle(annotated_image, (text_x, text_y - text_size[1] - 2),
                  (text_x + text_size[0], text_y + 2), (0, 0, 0), -1)
    cv2.putText(annotated_image, text, (text_x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

# ✅ Resize window for display (50% smaller)
scale = 0.7  # change to 0.3 for smaller, 0.7 for larger
new_width = int(annotated_image.shape[1] * scale)  
new_height = int(annotated_image.shape[0] * scale)
resized_image = cv2.resize(annotated_image, (new_width, new_height))

# Show the smaller image
cv2.imshow('Detections with Centroids', resized_image)
cv2.waitKey(0)
cv2.destroyAllWindows()