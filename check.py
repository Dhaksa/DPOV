'''import numpy as np
import cv2

def project_lidar_to_image(lidar_points, extrinsic, K, image):
    # Ensure lidar_points is Nx3
    lidar_points = np.atleast_2d(lidar_points)
    # Convert to homogeneous coordinates
    lidar_homo = np.hstack([lidar_points, np.ones((lidar_points.shape[0], 1))])  # Nx4
    # Transform to camera frame
    cam_points = (extrinsic @ lidar_homo.T).T  # Nx4
    cam_points = cam_points[:, :3]
    # Filter points in front of the camera
    front_mask = cam_points[:, 2] > 0
    cam_points = cam_points[front_mask]
    # Project to 2D
    proj_points = (K @ cam_points.T).T  # Nx3
    proj_points = proj_points / proj_points[:, 2:3]
    uvs = proj_points[:, :2].astype(int)
    # Draw points
    vis = image.copy()
    for (u, v) in uvs:
        if 0 <= u < vis.shape[1] and 0 <= v < vis.shape[0]:
            cv2.circle(vis, (u, v), 8, (0, 255, 0), -1)
    return vis

# --- Your calibration values ---
K = np.array([
    [591.77, 0, 306.61],
    [0, 601.06, 277.39],
    [0, 0, 1]
], dtype=np.float32)

T_final = np.array([
    [ 0.99529164, -0.02207938, -0.09437715, -0.04255894],
    [ 0.05053968,  0.94908151,  0.31095019,  0.14069764],
    [ 0.08270602, -0.31425592,  0.94572878,  0.42786594],
    [ 0.,          0.,          0.,          1.        ]
])

lidar_points = np.array([
    [0.477431, -0.134446, 0.0],
    [0.478584, -0.130280, 0.0],
    [0.477767, -0.125596, 0.0],
    [0.477874, -0.121182, 0.0],
    [0.477941, -0.116776, 0.0],
    [0.477967, -0.112378, 0.0],
    [0.477953, -0.107988, 0.0],
    [0.477898, -0.103607, 0.0],
    [0.477804, -0.099235, 0.0],
    [0.477670, -0.094873, 0.0],
    [0.478478, -0.090706, 0.0],
    [0.479250, -0.086533, 0.0],
    [0.478015, -0.082016, 0.0],
    [0.477725, -0.077686, 0.0],
    [0.477395, -0.073368, 0.0],
    [0.477027, -0.069062, 0.0],
    [0.477610, -0.064902, 0.0],
    [0.476174, -0.060486, 0.0],
    [0.476683, -0.056334, 0.0],
    [0.475167, -0.051960, 0.0],
    [0.476597, -0.047918, 0.0],
    [0.476997, -0.043763, 0.0],
    [0.476363, -0.039522, 0.0],
    [0.476690, -0.035369, 0.0],
    [0.475982, -0.031148, 0.0],
    [0.476235, -0.026999, 0.0],
    [0.475454, -0.022800, 0.0],
    [0.476633, -0.018695, 0.0],
    [0.476778, -0.014541, 0.0],
    [0.476887, -0.010385, 0.0],
    [0.474959, -0.006203, 0.0],
    [0.474996, -0.002064, 0.0],
    [0.473995,  0.002071, 0.0],
    [0.473959,  0.006202, 0.0],
    [0.473887,  0.010332, 0.0],
    [0.473779,  0.014461, 0.0],
    [0.473635,  0.018589, 0.0],
    [0.472457,  0.022668, 0.0],
    [0.474238,  0.026897, 0.0],
    [0.473985,  0.031029, 0.0],
    [0.473697,  0.035158, 0.0],
    [0.473373,  0.039285, 0.0],
    [0.473012,  0.043409, 0.0],
    [0.473611,  0.047629, 0.0],
    [0.473178,  0.051755, 0.0],
    [0.474695,  0.056111, 0.0],
    [0.475180,  0.060371, 0.0],
    [0.473645,  0.064375, 0.0],
    [0.474056,  0.068644, 0.0],
    [0.474428,  0.072924, 0.0],
    [0.480684,  0.078180, 0.0]
])

img = cv2.imread('/home/dhaksana/ros2_ws/src/ros2_camera_lidar_fusion/data/20250521_174117.png')
if img is None:
    raise FileNotFoundError("Camera image not found. Please check the path.")

# --- Project and visualize ---
vis_img = project_lidar_to_image(lidar_points, T_final, K, img)
cv2.imshow("LiDAR on Camera", vis_img)
cv2.waitKey(0)
cv2.destroyAllWindows()'''


import numpy as np
import cv2

# --- Camera Intrinsic Matrix ---
K = np.array([
    [591.77, 0, 306.61],
    [0, 601.06, 277.39],
    [0, 0, 1]
], dtype=np.float32)

# --- LiDAR-to-Camera Extrinsic Matrix ---
T_final = np.array([
    [ 0.99529164, -0.02207938, -0.09437715, -0.04255894],
    [ 0.05053968,  0.94908151,  0.31095019,  0.14069764],
    [ 0.08270602, -0.31425592,  0.94572878,  0.42786594],
    [ 0.,          0.,          0.,          1.        ]
])

# --- Your LiDAR Points (paste your list here) ---
lidar_points = np.array([
    [0.477431, -0.134446, 0.0],
    [0.478584, -0.130280, 0.0],
    [0.477767, -0.125596, 0.0],
    [0.477874, -0.121182, 0.0],
    [0.477941, -0.116776, 0.0],
    [0.477967, -0.112378, 0.0],
    [0.477953, -0.107988, 0.0],
    [0.477898, -0.103607, 0.0],
    [0.477804, -0.099235, 0.0],
    [0.477670, -0.094873, 0.0],
    [0.478478, -0.090706, 0.0],
    [0.479250, -0.086533, 0.0],
    [0.478015, -0.082016, 0.0],
    [0.477725, -0.077686, 0.0],
    [0.477395, -0.073368, 0.0],
    [0.477027, -0.069062, 0.0],
    [0.477610, -0.064902, 0.0],
    [0.476174, -0.060486, 0.0],
    [0.476683, -0.056334, 0.0],
    [0.475167, -0.051960, 0.0],
    [0.476597, -0.047918, 0.0],
    [0.476997, -0.043763, 0.0],
    [0.476363, -0.039522, 0.0],
    [0.476690, -0.035369, 0.0],
    [0.475982, -0.031148, 0.0],
    [0.476235, -0.026999, 0.0],
    [0.475454, -0.022800, 0.0],
    [0.476633, -0.018695, 0.0],
    [0.476778, -0.014541, 0.0],
    [0.476887, -0.010385, 0.0],
    [0.474959, -0.006203, 0.0],
    [0.474996, -0.002064, 0.0],
    [0.473995,  0.002071, 0.0],
    [0.473959,  0.006202, 0.0],
    [0.473887,  0.010332, 0.0],
    [0.473779,  0.014461, 0.0],
    [0.473635,  0.018589, 0.0],
    [0.472457,  0.022668, 0.0],
    [0.474238,  0.026897, 0.0],
    [0.473985,  0.031029, 0.0],
    [0.473697,  0.035158, 0.0],
    [0.473373,  0.039285, 0.0],
    [0.473012,  0.043409, 0.0],
    [0.473611,  0.047629, 0.0],
    [0.473178,  0.051755, 0.0],
    [0.474695,  0.056111, 0.0],
    [0.475180,  0.060371, 0.0],
    [0.473645,  0.064375, 0.0],
    [0.474056,  0.068644, 0.0],
    [0.474428,  0.072924, 0.0],
    [0.480684,  0.078180, 0.0]
])

# --- Load Camera Image ---
img = cv2.imread('/home/dhaksana/ros2_ws/src/ros2_camera_lidar_fusion/data/20250521_174117.png')
if img is None:
    raise FileNotFoundError("Camera image not found. Please check the path.")

def project_and_draw(lidar_points, extrinsic, K, img):
    lidar_points = np.atleast_2d(lidar_points)
    lidar_homo = np.hstack([lidar_points, np.ones((lidar_points.shape[0], 1))])
    cam_points = (extrinsic @ lidar_homo.T).T[:, :3]
    mask = cam_points[:, 2] > 0  # Only points in front of the camera
    cam_points = cam_points[mask]
    if cam_points.shape[0] == 0:
        print("No points are in front of the camera after transformation!")
        return img
    proj_points = (K @ cam_points.T).T
    proj_points = proj_points / proj_points[:, 2:3]
    uvs = proj_points[:, :2].astype(int)
    vis = img.copy()
    for (u, v) in uvs:
        if 0 <= u < vis.shape[1] and 0 <= v < vis.shape[0]:
            cv2.circle(vis, (u, v), 8, (0, 255, 0), -1)
    return vis

# --- Visualize ---
vis_img = project_and_draw(lidar_points, T_final, K, img)
cv2.imshow("LiDAR Points Projected on Camera", vis_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
