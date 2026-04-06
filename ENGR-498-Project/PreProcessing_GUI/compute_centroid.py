import numpy as np
import open3d as o3d
import random

def compute_wire_centroids(npz_path):
    data = np.load(npz_path, allow_pickle=True)

    wires = data["wires"].item()
    locations = wires["Location"]

    centroids = {}

    for i, wire_points in enumerate(locations):
        pts = np.array(wire_points, dtype=float)

        if pts.size == 0:
            continue

        # Compute centroid (mean of x, y, z)
        centroid = pts.mean(axis=0)

        # Store in dictionary
        centroids[f"wire_{i}"] = {
            "centroid": centroid.tolist(),  # convert to list for easy use / JSON later
            "num_points": len(pts)
        }

    return centroids


if __name__ == "__main__":
    npz_file = r"C:\Users\henry\OneDrive\Documents\GitHub\ENGR498-GUI-Project\wires_points.npz"

    centroids = compute_wire_centroids(npz_file)

    # Print results
    for wire_id, data in centroids.items():
        print(f"{wire_id}: centroid = {data['centroid']}, points = {data['num_points']}")

#     ###############################################################################
#####################################################################################

#the implementation below verifies compute centroid function by visualizing the centroid
#the implementation above computes the centroid and prints it out, but can be modified to 
#write the centroid to a json file

################################################################################
################################################################################

# import open3d as o3d
# import numpy as np
# import random


# def compute_wire_centroids(locations):
#     centroids = []

#     for wire_points in locations:
#         pts = np.array(wire_points, dtype=float)

#         if pts.size == 0:
#             centroids.append(None)
#             continue

#         centroid = pts.mean(axis=0)
#         centroids.append(centroid)

#     return centroids


# if __name__ == "__main__":
#     npz_file = r"C:\Users\henry\OneDrive\Documents\GitHub\ENGR498-GUI-Project\wires_points.npz"

#     data = np.load(npz_file, allow_pickle=True)
#     wires = data["wires"].item()
#     locations = wires["Location"]

#     centroids = compute_wire_centroids(locations)

#     geometries = []

#     for i, wire_points in enumerate(locations):
#         pts = np.array(wire_points, dtype=float)

#         if pts.size == 0:
#             continue

#         # --- Wire point cloud ---
#         wire_pc = o3d.geometry.PointCloud()
#         wire_pc.points = o3d.utility.Vector3dVector(pts)

#         color = [random.random(), random.random(), random.random()]
#         wire_pc.paint_uniform_color(color)

#         geometries.append(wire_pc)

#         # --- Centroid point ---
#         centroid = centroids[i]
#         if centroid is not None:
#             centroid_pc = o3d.geometry.PointCloud()
#             centroid_pc.points = o3d.utility.Vector3dVector([centroid])

#             # Bright red for centroid
#             centroid_pc.paint_uniform_color([1, 0, 0])

#             geometries.append(centroid_pc)

#     # Optional: make centroid points easier to see
#     vis = o3d.visualization.Visualizer()
#     vis.create_window()

#     for g in geometries:
#         vis.add_geometry(g)

#     render_option = vis.get_render_option()
#     render_option.point_size = 5.0  # increase point size (default ~1)

#     vis.run()
#     vis.destroy_window()