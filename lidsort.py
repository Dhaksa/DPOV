import numpy as np

# --- Read LiDAR points from text file (skip the Index column) ---
def read_lidar_points(filename):
    data = []
    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.lower().startswith("index"):
                continue
            parts = line.split()
            if len(parts) >= 4:
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                data.append([x, y, z])
    return np.array(data, dtype=np.float64)

lidar_points = read_lidar_points('/home/dhaksana/ros2_ws/src/ros2_camera_lidar_fusion/data/20250521_174117 sorted.txt')

# --- Randomly pick 16 unique points ---
if lidar_points.shape[0] < 16:
    raise ValueError("Not enough points in the file to pick 16 unique points.")

indices = np.random.choice(lidar_points.shape[0], size=16, replace=False)
selected_points = lidar_points[indices]

print("Selected 16 LiDAR points:\n", selected_points)
