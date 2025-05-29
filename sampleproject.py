import matplotlib
matplotlib.use('Agg')  # Avoids Qt errors in headless mode

import cv2
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
import os

# --- PARAMETERS ---
image_path = "/home/dhaksana/ros2_ws/src/ros2_camera_lidar_fusion/data/20250521_174117.png"          # Set your image file path
pcd_path = "/home/dhaksana/ros2_ws/src/ros2_camera_lidar_fusion/data/20250521_174117.pcd"       # Set your point cloud file path
output_dir = "output"                  # Directory to save results
checkerboard_size = (4,4)             # (columns, rows) of inner corners in checkerboard
square_size = 0.037                  # Size of a square in meters (adjust for your board)

os.makedirs(output_dir, exist_ok=True)

# --- 1. Detect Checkerboard Corners in the Image ---
image = cv2.imread(image_path)
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, None)

if not ret:
    print("Checkerboard not detected in image.")
else:
    cv2.drawChessboardCorners(image, checkerboard_size, corners, ret)
    cv2.imwrite(os.path.join(output_dir, "checkerboard_detected.png"), image)
    print("Checkerboard corners detected and saved.")

# --- 2. Load Point Cloud and Segment Checkerboard Region ---
pcd = o3d.io.read_point_cloud(pcd_path)
points = np.asarray(pcd.points)

# Filter out distant points (optional, adjust as needed)
distances = np.linalg.norm(points, axis=1)
mask = (distances > 0.1) & (distances < 5.0)
points_filtered = points[mask]

# Cluster points to find the checkerboard (DBSCAN)
clustering = DBSCAN(eps=0.05, min_samples=10).fit(points_filtered)
labels = clustering.labels_
unique_labels = np.unique(labels)
valid_labels = unique_labels[unique_labels != -1]
if len(valid_labels) == 0:
    print("No valid clusters found in point cloud.")
    checkerboard_points = None
else:
    # Take the largest cluster as checkerboard
    largest_label = max(valid_labels, key=lambda l: np.sum(labels == l))
    checkerboard_points = points_filtered[labels == largest_label]
    print(f"Checkerboard cluster found: {len(checkerboard_points)} points")

    # Save checkerboard points as PCD
    checkerboard_pcd = o3d.geometry.PointCloud()
    checkerboard_pcd.points = o3d.utility.Vector3dVector(checkerboard_points)
    o3d.io.write_point_cloud(os.path.join(output_dir, "checkerboard_points.pcd"), checkerboard_pcd)

# --- 3. Visualization (save as PNG, not display) ---
plt.figure(figsize=(8, 6))
plt.scatter(points_filtered[:, 0], points_filtered[:, 1], s=1, alpha=0.2, label='All points')
if checkerboard_points is not None:
    plt.scatter(checkerboard_points[:, 0], checkerboard_points[:, 1], s=5, c='r', label='Checkerboard')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.axis('equal')
plt.legend()
plt.title('Checkerboard Extraction (LiDAR)')
plt.savefig(os.path.join(output_dir, "checkerboard_extraction.png"))
print("Visualization saved.")

# --- 4. Print summary ---
if ret and checkerboard_points is not None:
    print("Both checkerboard corners and LiDAR cluster extracted successfully.")
else:
    print("Extraction incomplete. Check logs above.")

