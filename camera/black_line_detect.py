import cv2
import numpy as np

# Load image
# image_path = '/home/aldrin/ros2_ws/src/my_cobot_pro/camera/lab3.png'
image_path = '/home/aldrin/ros2_ws/src/my_cobot_pro/camera/lab3_edited.jpg'

image = cv2.imread(image_path)
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Invert and threshold to binary
_, binary = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)

# Thin the line to 1-pixel width (skeleton)
skeleton = cv2.ximgproc.thinning(binary)

# Get coordinates of skeleton pixels
ys, xs = np.where(skeleton == 255)
points = np.array(list(zip(xs, ys)))

# Use OpenCV contour finding on skeleton
skeleton = skeleton.astype(np.uint8)
contours, _ = cv2.findContours(skeleton, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

# Select the longest contour
contour = max(contours, key=lambda c: cv2.arcLength(c, False))

# Uniformly sample points along the centerline
curve = contour[:, 0, :]  # Shape (N, 2)
num_points = 15  # ← Change this as needed

# Compute cumulative length along the curve
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

# Draw sampled points on the original image
for pt in sampled_points:
    cv2.circle(image, pt, 3, (0, 0, 255), -1)

cv2.imshow("Centerline with Sampled Points", image)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Output cleaned coordinates
print("Sampled centerline coordinates:")
for pt in sampled_points:
    print(pt)
