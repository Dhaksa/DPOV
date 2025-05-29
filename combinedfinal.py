import numpy as np
from scipy.spatial.transform import Rotation as R

# Set numpy print options for full output
np.set_printoptions(precision=8, suppress=True, linewidth=200)

# Step 1: Define your 5 transformation matrices
T1 = np.array([[ 0.99515984, -0.02645739, -0.09464094, -0.0425442 ],
               [ 0.05479582,  0.94885289,  0.31092702,  0.14064886],
               [ 0.08157401, -0.314608  ,  0.94571004,  0.4277503 ],
               [ 0.        ,  0.        ,  0.        ,  1.        ]])

T2 = np.array([[ 0.99542301, -0.01621593, -0.09418109, -0.0425721 ],
               [ 0.04488722,  0.94936089,  0.31096469,  0.14074109],
               [ 0.08436926, -0.31376893,  0.94574356,  0.42794939],
               [ 0.        ,  0.        ,  0.        ,  1.        ]])

T3 = np.array([[ 0.99533359, -0.02166021, -0.09403128, -0.0425788 ],
               [ 0.05003255,  0.94910027,  0.31097494,  0.14076345],
               [ 0.08250933, -0.31422843,  0.9457551 ,  0.42801675],
               [ 0.        ,  0.        ,  0.        ,  1.        ]])

T4 = np.array([[ 0.99520445, -0.02561843, -0.09440233, -0.04255488],
               [ 0.0539227 ,  0.94889348,  0.31095579,  0.14068421],
               [ 0.08161155, -0.31455502,  0.94572443,  0.4278577 ],
               [ 0.        ,  0.        ,  0.        ,  1.        ]])

T5 = np.array([[ 0.99530256, -0.02044407, -0.09463005, -0.04254474],
               [ 0.04905836,  0.94916669,  0.31092745,  0.14065058],
               [ 0.08346307, -0.31410928,  0.94571099,  0.42775554],
               [ 0.        ,  0.        ,  0.        ,  1.        ]])

T_all = np.stack([T1, T2, T3, T4, T5])

# Step 2: Extract rotations and translations
rot_mats = T_all[:, :3, :3]    # Shape: (5, 3, 3)
translations = T_all[:, :3, 3] # Shape: (5, 3)

# Step 3: Convert rotations to quaternions and average
rots = R.from_matrix(rot_mats)
quats = rots.as_quat()  # Shape: (5, 4), format: [x, y, z, w]

# Average quaternions (simple mean, then normalize)
quat_mean = np.mean(quats, axis=0)
quat_mean /= np.linalg.norm(quat_mean)

# Convert mean quaternion back to rotation matrix
rot_final = R.from_quat(quat_mean).as_matrix()

# Step 4: Average translations
trans_final = np.mean(translations, axis=0)

# Step 5: Construct the final 4x4 extrinsic matrix
T_final = np.eye(4)
T_final[:3, :3] = rot_final
T_final[:3, 3] = trans_final

print("Final LiDAR-to-Camera Extrinsic Matrix:")
print(T_final)

# Step 6: (Optional) Transform a LiDAR point to the camera frame
lidar_point = np.array([1.2, 0.5, 0.3, 1.0])  # Example point in homogeneous coords
camera_point = T_final @ lidar_point
print("\nTransformed LiDAR point in camera frame:")
print(camera_point[:3])


'''output:

Final LiDAR-to-Camera Extrinsic Matrix:
[[ 0.99529164 -0.02207938 -0.09437715 -0.04255894]
 [ 0.05053968  0.94908151  0.31095019  0.14069764]
 [ 0.08270602 -0.31425592  0.94572878  0.42786594]
 [ 0.          0.          0.          1.        ]]

Transformed LiDAR point in camera frame:
[1.11243819 0.76917107 0.65370384]

'''