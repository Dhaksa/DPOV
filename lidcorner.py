'''import numpy as np
import glob
import os
import pickle

# Set your folder path
lidar_folder = '/home/dhaksana/ros2_ws/src/ros2_camera_lidar_fusion/data'  # <-- Change this if needed
txt_paths = sorted(glob.glob(os.path.join(lidar_folder, '*.txt')))

all_lidar_points = []

for txt_path in txt_paths:
    lidar_points = []
    with open(txt_path, 'r') as f:
        for line in f:
            nums = line.strip().split()
            if len(nums) == 3:
                xyz = [float(x) for x in nums]
                lidar_points.append(xyz)
    lidar_points = np.array(lidar_points)  # shape: (N, 3)
    # Sort by y, then x (row-wise ordering)
    if lidar_points.shape[0] > 0:
        lidar_points = lidar_points[np.lexsort((lidar_points[:,0], lidar_points[:,1]))]
    print(f"{os.path.basename(txt_path)}: {lidar_points.shape}")
    all_lidar_points.append(lidar_points)

# Save as a list of arrays (since each file may have a different number of points)
with open('lidcorners.pkl', 'wb') as f:
    pickle.dump(all_lidar_points, f)

print("Saved all LiDAR checkerboard points to lidar_corners.pkl")'''



import glob
import os

# Optional: for natural sorting (file_1, file_2, ..., file_10)
import re

def natural_sort_key(s):
    # Helper for human-like sorting: file_2.txt before file_10.txt
    return [int(text) if text.isdigit() else text.lower()
            for text in re.split('(\d+)', s)]

# Folder containing your sorted LiDAR .txt files
lidar_folder = '/home/dhaksana/ros2_ws/src/ros2_camera_lidar_fusion/data'
txt_paths = glob.glob(os.path.join(lidar_folder, '*sorted.txt'))
txt_paths = sorted(txt_paths, key=natural_sort_key)

with open('all_lidar_points.txt', 'w') as outfile:
    for txt_path in txt_paths:
        base = os.path.basename(txt_path)
        outfile.write(f"# {base}\n")
        with open(txt_path, 'r') as infile:
            for line in infile:
                outfile.write(line)
        outfile.write('\n')  # Blank line between files

print("All LiDAR points saved to all_lidar_points.txt")


