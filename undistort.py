import cv2
import numpy as np
import os

# === Calibration parameters (replace with your actual values) ===
K = np.array([
    [648.227652, 0.0, 325.995156],
    [0.0, 643.799313, 237.972044],
    [0.0, 0.0, 1.0]
])
dist = np.array([0.081559, -0.159391, 0.010664, 0.000494, 0.000000])

CHECKERBOARD = (4,4)  # (columns, rows) of internal corners

input_dir = 'distort3'      # <-- Change this to your folder
output_dir = 'undistorted_annotated' # <-- Output folder
os.makedirs(output_dir, exist_ok=True)

images = sorted([os.path.join(input_dir, f) for f in os.listdir(input_dir)
                 if f.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp'))])

found_count = 0

for fname in images:
    img = cv2.imread(fname)
    if img is None:
        print(f"Could not read {fname}. Skipping.")
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    print(f"Processing {fname} - Checkerboard found: {ret}")

    if ret:
        found_count += 1

        # Undistort the image
        h, w = img.shape[:2]
        new_K, _ = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), 1, (w, h))
        undistorted = cv2.undistort(img, K, dist, None, new_K)

        # Undistort the corner points for correct overlay
        undist_corners = cv2.undistortPoints(corners, K, dist, P=new_K)
        undist_corners = undist_corners.reshape(-1, 1, 2)
        img_vis = cv2.drawChessboardCorners(undistorted.copy(), CHECKERBOARD, undist_corners, ret)

        # Save the undistorted, annotated image
        out_path = os.path.join(output_dir, os.path.basename(fname))
        cv2.imwrite(out_path, img_vis)
    else:
        print("Corners not found — check orientation or pattern.")

print(f"\nTotal images with detected checkerboard corners: {found_count} out of {len(images)}")
print(f"Undistorted images with checkerboard overlay are saved in: {output_dir}")


