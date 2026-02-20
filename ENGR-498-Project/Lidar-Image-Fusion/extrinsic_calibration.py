import cv2
import numpy as np

# Load intrinsics
data = np.load("camera_intrinsics.npz")
K = data["K"]
dist = data["dist"]

# Manually measured 3D points from LiDAR (meters)
# Example: 6–10 checkerboard corners
pts_3d = np.array([
    [0.0, 0.0, 0.0],
    [0.025, 0.0, 0.0],
    [0.0, 0.025, 0.0],
    [0.025, 0.025, 0.0],
], dtype=np.float32)

# Corresponding 2D image points
pts_2d = np.array([
    [523, 412],
    [611, 410],
    [525, 498],
    [613, 495],
], dtype=np.float32)

ret, rvec, tvec = cv2.solvePnP(
    pts_3d, pts_2d, K, dist
)

R, _ = cv2.Rodrigues(rvec)

np.savez("extrinsics.npz", R=R, t=tvec)
