import numpy as np
import glob
import os
import matplotlib.pyplot as plt
from skimage.measure import LineModelND, ransac

def load_points(txt_file):
    # Skip the first row assuming it contains headers like 'Index' or column names
    return np.loadtxt(txt_file, skiprows=1)

lidar_folder = '/home/dhaksana/ros2_ws/src/ros2_camera_lidar_fusion/data'
txt_paths = glob.glob(os.path.join(lidar_folder, '*sorted.txt'))

for txt_file in txt_paths:
    try:
        points = load_points(txt_file)
    except Exception as e:
        print(f"Error loading {txt_file}: {e}")
        continue

    if points.shape[0] < 2:
        print(f"Not enough points in {txt_file}")
        continue

    # RANSAC line fitting using skimage
    model_robust, inliers = ransac(
        points, LineModelND, min_samples=2, residual_threshold=0.01, max_trials=1000
    )
    outliers = ~inliers

    # Generate line for plotting
    x_vals = np.linspace(points[:, 0].min(), points[:, 0].max(), 100)
    y_vals_robust = model_robust.predict_y(x_vals)

    # Print fitted line parameters
    # For LineModelND, the line is parameterized as point + direction * t
    origin = model_robust.params[0]
    direction = model_robust.params[1]
    print(f"File: {os.path.basename(txt_file)} | Line origin: {origin}, direction: {direction} | Inliers: {np.sum(inliers)}")

    # Plot
    plt.figure(figsize=(8, 6))
    plt.scatter(points[outliers, 0], points[outliers, 1], color='red', label='Outliers', s=10)
    plt.scatter(points[inliers, 0], points[inliers, 1], color='blue', label='Inliers', s=10)
    plt.plot(x_vals, y_vals_robust, 'g-', label='RANSAC Fitted Line', linewidth=2)
    plt.title(f'RANSAC Line Fit: {os.path.basename(txt_file)}')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()