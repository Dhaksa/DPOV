import numpy as np
import cv2

# --- Camera parameters ---
camera_matrix = np.array([
    [591.768444, 0.0, 306.608469],
    [0.0, 601.056316, 277.394428],
    [0.0, 0.0, 1.0]
], dtype=np.float64)

dist_coeffs = np.array([0.090388, -0.012063, 0.018302, 0.005220, 0.0], dtype=np.float64)

# --- Read LiDAR 3D points (one point per line: x y z) ---
lidar_points = np.loadtxt('/home/dhaksana/ros2_ws/src/ros2_camera_lidar_fusion/data/lid1points.txt', dtype=np.float64)

# --- Read camera 2D checkerboard points ---
camera_points = np.loadtxt('/home/dhaksana/ros2_ws/src/ros2_camera_lidar_fusion/data/image1campoints.txt', dtype=np.float64)

# --- Check and prepare data ---
if lidar_points.shape[0] != camera_points.shape[0]:
    raise ValueError(f"Number of LiDAR points ({lidar_points.shape[0]}) and camera points ({camera_points.shape[0]}) must match.")
if lidar_points.shape[0] < 4:
    raise ValueError("At least 4 corresponding points are required for solvePnP.")

lidar_points = np.ascontiguousarray(lidar_points, dtype=np.float64)
camera_points = np.ascontiguousarray(camera_points, dtype=np.float64)

# --- Load the image ---
image = cv2.imread('/home/dhaksana/ros2_ws/src/ros2_camera_lidar_fusion/data/20250521_174117.png')
if image is None:
    raise FileNotFoundError("Image file not found.")

# --- Estimate extrinsic parameters (rotation and translation) ---
success, rvec, tvec = cv2.solvePnP(
    lidar_points, camera_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE
)
if not success:
    raise RuntimeError("Extrinsic calibration failed. Check your correspondences and data.")

# --- Project LiDAR 3D points onto the image ---
projected_lidar_points, _ = cv2.projectPoints(
    lidar_points, rvec, tvec, camera_matrix, dist_coeffs
)
projected_lidar_points = projected_lidar_points.squeeze()

# --- Draw projected LiDAR points (green) and camera points (red) ---
for pt in projected_lidar_points:
    cv2.circle(image, tuple(np.round(pt).astype(int)), 6, (0, 255, 0), -1)  # Green

for pt in camera_points:
    cv2.circle(image, tuple(np.round(pt).astype(int)), 6, (0, 0, 255), -1)  # Red

# --- Show the result ---
cv2.imshow('LiDAR (green) and Camera (red) Checkerboard Points', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
