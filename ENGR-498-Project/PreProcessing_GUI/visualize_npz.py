import open3d as o3d
import numpy as np
import random

data = np.load(r"C:\Users\henry\OneDrive\Documents\GitHub\ENGR498-GUI-Project\wires_points.npz", allow_pickle=True)
#C:\Users\henry\Downloads\law2-matched-filtered-classifier-powerline-flainet\law2_matched_filtered_-_classifier_-_powerline_flainet\ground_points.npz")
#C:\Users\henry\OneDrive\Documents\GitHub\ENGR498-GUI-Project\wires_points.npz

# ##############visualize wires##################### This section works to visualize the wires

wires = data["wires"].item()

locations = wires["Location"]   # list of 14 wires

pcds = []

for wire_points in locations:

    # convert MATLAB array → numpy array
    pts = np.array(wire_points, dtype=float)

    if pts.size == 0:
        continue

    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(pts)

    color = [random.random(), random.random(), random.random()]
    pc.paint_uniform_color(color)

    pcds.append(pc)

o3d.visualization.draw_geometries(pcds)

##############visualize wires#####################


# wires_obj = data["wires"]

# print(type(wires_obj))      # numpy.ndarray
# print(wires_obj.shape)      # ()

# wires = wires_obj.item()    # <-- THIS extracts the dict

# print(type(wires))          # dict
# print(wires.keys())         # should show your 14 wires


# print(type(wires["Location"]))
# print(type(wires["Label"]))
# print(len(wires["Location"]))
# print(len(wires["Label"]))
# print(wires["Ids"])

# print(data.files)
# keys = sorted(data.files, key=lambda k: int(k.split('_')[1]))
# #wire = data["location"]
# print("Keys in NPZ:", data.files)

# # Load ground points
# ground = data["ground_points"]

# print("Ground shape:", ground.shape)
# print("Ground dtype:", ground.dtype)

# Create Open3D point cloud
# pcd_ground = o3d.geometry.PointCloud()
# pcd_ground.points = o3d.utility.Vector3dVector(ground.astype(np.float64))
# pcd_ground.paint_uniform_color([0, 1, 0])  # green

# # Show it
# o3d.visualization.draw_geometries([pcd_ground])