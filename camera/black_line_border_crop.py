import cv2
import numpy as np

# Load image
image_path = '/home/aldrin/ros2_ws/src/my_cobot_pro/camera/lab3.jpg'
image = cv2.imread(image_path)
h, w = image.shape[:2]

# Define center region (you can adjust scale here)
center_margin = 0.05  # 25% margin around center
x1 = int(w * center_margin)
x2 = int(w * (1 - center_margin))
y1 = int(h * center_margin)
y2 = int(h * (1 - center_margin))
center_crop = image[y1:y2, x1:x2]

# Convert center crop to HSV for robust color segmentation
hsv_crop = cv2.cvtColor(center_crop, cv2.COLOR_BGR2HSV)

# Gray detection in real images (low saturation, mid brightness)
lower_hsv = np.array([0, 0, 30])     # Lower value and saturation
upper_hsv = np.array([180, 70, 150]) # Slightly higher saturation and value


# Generate mask
mask = cv2.inRange(hsv_crop, lower_hsv, upper_hsv)

# Morphological cleanup
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))

# Skeletonize
skeleton = cv2.ximgproc.thinning(mask)

# Find contours on skeleton
skeleton_uint8 = skeleton.astype(np.uint8)
contours, _ = cv2.findContours(skeleton_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

if not contours:
    raise RuntimeError("No contours found. Check color range or image quality.")

# Select the longest contour
contour = max(contours, key=lambda c: cv2.arcLength(c, False))
curve = contour[:, 0, :]  # Shape (N, 2)

# Uniformly sample points along the contour
num_points = 15
lengths = np.cumsum(np.sqrt(np.sum(np.diff(curve, axis=0)**2, axis=1)))
lengths = np.insert(lengths, 0, 0)
total_length = lengths[-1]
even_spacing = np.linspace(0, total_length, num_points)

sampled_points = []
for dist in even_spacing:
    idx = np.searchsorted(lengths, dist)
    idx = min(idx, len(curve) - 2)
    p1, p2 = curve[idx], curve[idx + 1]
    segment_length = lengths[idx + 1] - lengths[idx]
    if segment_length == 0:
        sampled_points.append(tuple(p1))
        continue
    ratio = (dist - lengths[idx]) / segment_length
    interpolated = (1 - ratio) * p1 + ratio * p2
    sampled_points.append(tuple(map(int, interpolated)))

# Convert sampled points from crop-relative to original image coordinates
sampled_points_original = [(pt[0] + x1, pt[1] + y1) for pt in sampled_points]

# Draw sampled points on original image
for pt in sampled_points_original:
    cv2.circle(image, pt, 3, (0, 0, 255), -1)

cv2.imshow("Centerline with Sampled Points", image)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Print sampled coordinates
print("Sampled centerline coordinates (original image):")
for pt in sampled_points_original:
    print(pt)
