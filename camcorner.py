'''import cv2
import numpy as np
import glob
import os

# Set your checkerboard pattern size (number of inner corners per row and column)
pattern_size = (4,4)  # Example: 7x6 inner corners; adjust to your checkerboard

# Path to your images
image_folder = '/home/dhaksana/ros2_ws/src/ros2_camera_lidar_fusion/data'
image_paths = sorted(glob.glob(os.path.join(image_folder, '*.png')))

all_corners = []

for img_path in image_paths:
    img = cv2.imread(img_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
    if ret:
        # Refine corner locations
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        all_corners.append(corners.squeeze())  # shape: (N, 2)
        # Optional: visualize
        vis = img.copy()
        cv2.drawChessboardCorners(vis, pattern_size, corners, ret)
        cv2.imshow('Corners', vis)
        cv2.waitKey(200)
    else:
        print(f"Checkerboard not found in {img_path}")

cv2.destroyAllWindows()
np.save('camera_corners.npy', all_corners)'''

import cv2
import numpy as np
import glob
import os

# Set your checkerboard pattern size (number of inner corners per row and column)
pattern_size = (4, 4)  # Adjust to your checkerboard

# Path to your images
image_folder = '/home/dhaksana/ros2_ws/src/ros2_camera_lidar_fusion/data'
image_paths = sorted(glob.glob(os.path.join(image_folder, '*.png')))

all_corners = []

# Open the output file once
with open('all_camera_corners.txt', 'w') as fout:
    for img_path in image_paths:
        img = cv2.imread(img_path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
        if ret:
            # Refine corner locations
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            corners = corners.squeeze()  # shape: (N, 2)
            all_corners.append(corners)
            # Write header and corners to file
            fout.write(f"# {os.path.basename(img_path)}\n")
            np.savetxt(fout, corners, fmt="%.6f")
            fout.write("\n")
        else:
            print(f"Checkerboard not found in {img_path}")

print("Saved all camera corners to all_camera_corners.txt")



