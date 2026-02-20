import numpy as np
import cv2
import open3d as o3d

# Load data
cloud = o3d.io.read_point_cloud("cloud.ply")
points = np.asarray(cloud.points)

img = cv2.imread("image.jpg")
h, w, _ = img.shape

intr = np.load("camera_intrinsics.npz")
ext = np.load("extrinsics.npz")

K = intr["K"]
R = ext["R"]
t = ext["t"]

# Transform LiDAR → camera frame
pts_cam = (R @ points.T + t).T

# Keep points in front of camera
mask = pts_cam[:, 2] > 0
pts_cam = pts_cam[mask]
pts_lidar = points[mask]

# Project
proj = (K @ (pts_cam.T / pts_cam[:, 2])).T
uv = proj[:, :2].astype(int)

colors = []
for (u, v) in uv:
    if 0 <= u < w and 0 <= v < h:
        colors.append(img[v, u] / 255.0)
    else:
        colors.append([0, 0, 0])

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(pts_lidar)
pcd.colors = o3d.utility.Vector3dVector(colors)

o3d.visualization.draw_geometries([pcd])

