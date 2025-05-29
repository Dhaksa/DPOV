'''import numpy as np
import cv2

# Known checkerboard geometry (4x4 corners, 37mm squares)
square_size = 0.037  # meters
pattern_size = (4, 4)
objp = np.zeros((pattern_size[0]*pattern_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * square_size

# 2D image points (from camera, detected with cv2.findChessboardCorners)
img_points = np.array([ 
    [170.498413, 179.136810],
    [215.079498, 184.799072],
    [258.601532, 190.506943],
    [301.423920, 196.126129],
    [162.824081, 220.284485],
    [208.696686, 225.776703],
    [253.558975, 231.302948],
    [297.492096, 236.610504],
    [154.542160, 263.400848],
    [201.907806, 268.699524],
    [248.247284, 273.960510],
    [293.438843, 279.419891],
    [146.232269, 310.105103],
    [194.779648, 315.032135],
    [242.351303, 320.024994],
    [288.998413, 324.648956]
], dtype=np.float32)

# Reshape img_points to (N, 1, 2) as required by solvePnP
img_points = img_points.reshape(-1, 1, 2)

# Camera intrinsics (example values, use your calibration)
K = np.array([
    [591.77, 0, 306.61],
    [0, 601.06, 277.39],
    [0, 0, 1]
], dtype=np.float32)
D = np.zeros(5)  # Or your actual distortion coefficients

# Estimate checkerboard pose in camera frame
success, rvec_cb, tvec_cb = cv2.solvePnP(objp, img_points, K, D)
if not success:
    raise RuntimeError("solvePnP failed!")

# Print the rotation and translation vectors
print("Rotation Vector (rvec):\n", rvec_cb)
print("Translation Vector (tvec):\n", tvec_cb)'''

'''
import numpy as np
import cv2
from scipy.optimize import minimize

# --- Step 1: Load LiDAR Points ---
lidar_data = """
0.477431	-0.134446	0.0
0.478584	-0.130280	0.0
0.477767	-0.125596	0.0
0.477874	-0.121182	0.0
0.477941	-0.116776	0.0
0.477967	-0.112378	0.0
0.477953	-0.107988	0.0
0.477898	-0.103607	0.0
0.477804	-0.099235	0.0
0.477670	-0.094873	0.0
0.478478	-0.090706	0.0
0.479250	-0.086533	0.0
0.478015	-0.082016	0.0
0.477725	-0.077686	0.0
0.477395	-0.073368	0.0
0.477027	-0.069062	0.0
0.477610	-0.064902	0.0
0.476174	-0.060486	0.0
0.476683	-0.056334	0.0
0.475167	-0.051960	0.0
0.476597	-0.047918	0.0
0.476997	-0.043763	0.0
0.476363	-0.039522	0.0
0.476690	-0.035369	0.0
0.475982	-0.031148	0.0
0.476235	-0.026999	0.0
0.475454	-0.022800	0.0
0.476633	-0.018695	0.0
0.476778	-0.014541	0.0
0.476887	-0.010385	0.0
0.474959	-0.006203	0.0
0.474996	-0.002064	0.0
0.473995	0.002071	0.0
0.473959	0.006202	0.0
0.473887	0.010332	0.0
0.473779	0.014461	0.0
0.473635	0.018589	0.0
0.472457	0.022668	0.0
0.474238	0.026897	0.0
0.473985	0.031029	0.0
0.473697	0.035158	0.0
0.473373	0.039285	0.0
0.473012	0.043409	0.0
0.473611	0.047629	0.0
0.473178	0.051755	0.0
0.474695	0.056111	0.0
0.475180	0.060371	0.0
0.473645	0.064375	0.0
0.474056	0.068644	0.0
0.474428	0.072924	0.0
0.480684	0.078180	0.0
"""
lidar_points = np.loadtxt(lidar_data.strip().split('\n'))

# --- Step 2: Checkerboard Pose in Camera Frame (example values, use your solvePnP output) ---
# These should come from your cv2.solvePnP call
rvec_cb = np.array([[-0.32143393], [-0.07918268], [0.10345004]])
tvec_cb = np.array([[-0.11389072], [-0.08058134], [0.49368785]])

# --- Step 3: Compute Checkerboard Plane in Camera Frame ---
R_cb, _ = cv2.Rodrigues(rvec_cb)
normal_cb = R_cb @ np.array([0, 0, 1])
point_on_plane_cb = tvec_cb.flatten()
d_cb = -normal_cb.dot(point_on_plane_cb)
plane_cb = np.append(normal_cb, d_cb)  # [a, b, c, d] for ax+by+cz+d=0

# --- Step 4: Optimize LiDAR-to-Camera Transformation ---
def transform_points(points, rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    return points @ R.T + tvec.reshape(1, 3)

def point_to_plane_error(params, lidar_points, plane):
    rvec = params[:3]
    tvec = params[3:6]
    transformed = transform_points(lidar_points, rvec, tvec)
    num = np.abs(transformed @ plane[:3] + plane[3])
    denom = np.linalg.norm(plane[:3])
    return np.mean(num / denom)

x0 = np.zeros(6)  # Initial guess: [rvec_x, rvec_y, rvec_z, t_x, t_y, t_z]
res = minimize(point_to_plane_error, x0, args=(lidar_points, plane_cb), method='BFGS')
rvec_lidar2cam = res.x[:3]
tvec_lidar2cam = res.x[3:6]

# --- Step 5: Results ---
print("LiDAR to Camera rotation vector (Rodrigues):", rvec_lidar2cam)
print("LiDAR to Camera translation vector:", tvec_lidar2cam)
print("Mean point-to-plane error (meters):", point_to_plane_error(res.x, lidar_points, plane_cb))

# --- Optional: Transform LiDAR points to Camera Frame for Visualization ---
lidar_points_in_cam = transform_points(lidar_points, rvec_lidar2cam, tvec_lidar2cam)
print("Transformed LiDAR points (first 5):\n", lidar_points_in_cam[:5])'''



import numpy as np
import cv2

# Your calibration results
rvec = np.array([-0.31836715 ,-0.09071307 , 0.03540158])
tvec = np.array([-0.04254474 , 0.14065058 , 0.42775554])

# Convert rotation vector to rotation matrix
R, _ = cv2.Rodrigues(rvec)  # R is 3x3

# Build the 4x4 transformation matrix
T = np.eye(4)
T[:3, :3] = R
T[:3, 3] = tvec

print("LiDAR-to-Camera 4x4 transformation matrix:\n", T)

