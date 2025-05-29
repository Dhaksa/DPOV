import numpy as np
from sklearn.decomposition import PCA
from scipy.optimize import minimize
import cv2
import matplotlib.pyplot as plt

# --- 1. LiDAR random points from checkerboard (meters) ---
lidar_points = np.array([
    [0.477431, -0.134446, 0.0],
    [0.476655, -0.129755, 0.0],
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
    [0.477027, -0.069062, 0.0]
], dtype=np.float32)

# --- 2. Fit a plane to the LiDAR points (PCA) ---
def fit_plane(points):
    pca = PCA(n_components=3)
    pca.fit(points)
    normal = pca.components_[-1]
    point_on_plane = pca.mean_
    d = -point_on_plane.dot(normal)
    plane = np.append(normal, d)
    return plane  # [a, b, c, d] for ax + by + cz + d = 0

plane = fit_plane(lidar_points)
print("LiDAR plane equation: ax + by + cz + d = 0 ->", plane)

# --- 3. Camera checkerboard corners (2D image points, pixels) ---
camera_points_2d = np.array([
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

# --- 4. Generate 3D camera checkerboard corners (in checkerboard frame) ---
# For a 4x4 checkerboard with 30mm squares:
square_size = 0.03  # meters
pattern_size = (4, 4)
objp = np.zeros((pattern_size[0]*pattern_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * square_size
camera_corners_3d = objp  # (16,3)

# --- 5. Optimize extrinsic parameters (rotation/translation) by minimizing point-to-plane distance ---
def transform_points(points, rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    return points @ R.T + tvec.reshape(1, 3)

def point_to_plane_error(params, camera_points, plane):
    rvec = params[:3]
    tvec = params[3:6]
    transformed = transform_points(camera_points, rvec, tvec)
    num = np.abs(transformed @ plane[:3] + plane[3])
    denom = np.linalg.norm(plane[:3])
    return np.mean(num / denom)

x0 = np.zeros(6)  # Initial guess: rvec=0, tvec=0
res = minimize(point_to_plane_error, x0, args=(camera_corners_3d, plane), method='BFGS')
rvec_opt = res.x[:3]
tvec_opt = res.x[3:6]
print("Optimized rotation vector (Rodrigues):", rvec_opt)
print("Optimized translation vector:", tvec_opt)
print("Mean point-to-plane error (meters):", point_to_plane_error(res.x, camera_corners_3d, plane))

# --- 6. (Optional) Visualize transformed camera corners and LiDAR points ---
try:
    from mpl_toolkits.mplot3d import Axes3D
    transformed_corners = transform_points(camera_corners_3d, rvec_opt, tvec_opt)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(lidar_points[:,0], lidar_points[:,1], lidar_points[:,2], c='g', label='LiDAR points')
    ax.scatter(transformed_corners[:,0], transformed_corners[:,1], transformed_corners[:,2], c='r', marker='x', label='Transformed camera corners')
    ax.set_title('LiDAR points and transformed camera checkerboard corners')
    ax.legend()
    plt.show()
except Exception as e:
    print("3D visualization skipped (matplotlib 3D error):", e)