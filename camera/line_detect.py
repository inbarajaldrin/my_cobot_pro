import cv2
from ultralytics import YOLO

# Load trained OBB model
model = YOLO('/root/ros2_ws/src/my_cobot_pro/dataset/yolov8n-obb.pt').to('cpu')

# Load image from file
image_path = '/root/ros2_ws/src/my_cobot_pro/dataset/black line detection.v2i.yolov8-obb/test/images/rotated_rect_043_png.rf.81854ff1ff965736fcb46f26ba54f74d.jpg'  # Replace with your image path

image = cv2.imread(image_path)

# Run inference
results = model(image, conf=0.1)[0]  # conf threshold can be adjusted

# Draw OBB bounding boxes
for obb in results.obb:
    if obb is None:
        continue
    for box in obb.xywhr.cpu().numpy():
        x, y, w, h, angle = box
        rect = ((x, y), (w, h), angle)
        box_points = cv2.boxPoints(rect)
        box_points = box_points.astype(int)
        cv2.drawContours(image, [box_points], 0, (0, 255, 0), 2)

# Save annotated image
output_path = '/root/ros2_ws/src/my_cobot_pro/dataset/output_annotated.jpg'
cv2.imwrite(output_path, image)
print(f"Annotated image saved to {output_path}")
