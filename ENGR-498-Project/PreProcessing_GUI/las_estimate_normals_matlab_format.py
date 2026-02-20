#!/usr/bin/env python3
"""
LAS Normal Estimation Tool - MATLAB Compatible Output

Estimates normal vectors for LAS point cloud files and saves them in RGB channels
for MATLAB compatibility (same as the working reference file).

Requires: laspy, numpy, open3d
Install with: pip install laspy numpy open3d
"""

import sys
import os
import numpy as np
from tkinter import Tk, filedialog, messagebox

try:
    import laspy
    import open3d as o3d
except ImportError as e:
    print(f"Error: Missing required library - {e}")
    print("Install dependencies with: pip install laspy numpy open3d")
    input("Press Enter to exit...")
    sys.exit(1)


def select_input_file():
    """Open file dialog to select input LAS file."""
    root = Tk()
    root.withdraw()
    root.attributes('-topmost', True)
    
    file_path = filedialog.askopenfilename(
        title="Select LAS file for normal estimation",
        filetypes=[("LAS files", "*.las"), ("All files", "*.*")]
    )
    
    root.destroy()
    return file_path


def select_output_file(default_name):
    """Open file dialog to select output LAS file location."""
    root = Tk()
    root.withdraw()
    root.attributes('-topmost', True)
    
    file_path = filedialog.asksaveasfilename(
        title="Save MATLAB-compatible LAS file as",
        defaultextension=".las",
        initialfile=default_name,
        filetypes=[("LAS files", "*.las"), ("All files", "*.*")]
    )
    
    root.destroy()
    return file_path


def estimate_normals_matlab_compatible(input_las_path, output_las_path, search_radius=0.1, max_nn=30):
    """
    Estimate normals for a LAS file and save in MATLAB-compatible format.
    Normals are stored in RGB channels to match the working reference file.
    Output format: LAS 1.2, Point Format 3 (XYZ + GPS Time + RGB)
    
    Args:
        input_las_path: Path to input LAS file
        output_las_path: Path to output LAS file with normals in RGB
        search_radius: Radius for normal estimation (in same units as point cloud)
        max_nn: Maximum number of neighbors to use for normal estimation
    
    Returns:
        bool: True if successful, False otherwise
    """
    print(f"Reading LAS file: {input_las_path}")
    
    # Read the LAS file
    try:
        las_in = laspy.read(input_las_path)
    except Exception as e:
        print(f"Error reading LAS file: {e}")
        import traceback
        traceback.print_exc()
        messagebox.showerror("Error", f"Could not read LAS file:\n{e}")
        return False
    
    num_points = len(las_in.points)
    print(f"Number of points: {num_points:,}")
    
    if num_points == 0:
        print("Error: LAS file contains no points")
        messagebox.showerror("Error", "LAS file contains no points")
        return False
    
    # Extract XYZ coordinates
    points = np.vstack([las_in.x, las_in.y, las_in.z]).T
    
    print(f"Point cloud bounds:")
    print(f"  X: {points[:, 0].min():.3f} to {points[:, 0].max():.3f}")
    print(f"  Y: {points[:, 1].min():.3f} to {points[:, 1].max():.3f}")
    print(f"  Z: {points[:, 2].min():.3f} to {points[:, 2].max():.3f}")
    
    # Create Open3D point cloud
    print("\nCreating Open3D point cloud...")
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
    
    # Estimate normals
    print(f"\nEstimating normals...")
    print(f"  Search radius: {search_radius}")
    print(f"  Max neighbors: {max_nn}")
    print("  (This may take a while for large point clouds...)")
    
    try:
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=search_radius,
                max_nn=max_nn
            )
        )
        print("✓ Normal estimation completed")
    except Exception as e:
        print(f"Error estimating normals: {e}")
        import traceback
        traceback.print_exc()
        messagebox.showerror("Error", f"Could not estimate normals:\n{e}")
        return False
    
    # Orient normals consistently (optional but recommended)
    print("\nOrienting normals consistently...")
    try:
        pcd.orient_normals_consistent_tangent_plane(k=15)
        print("✓ Normals oriented consistently")
    except Exception as e:
        print(f"Warning: Could not orient normals consistently: {e}")
        print("  Continuing with unoriented normals...")
    
    # Extract normals
    normals = np.asarray(pcd.normals, dtype=np.float32)
    
    print(f"\nNormal vector statistics:")
    print(f"  Normal X range: {normals[:, 0].min():.4f} to {normals[:, 0].max():.4f}")
    print(f"  Normal Y range: {normals[:, 1].min():.4f} to {normals[:, 1].max():.4f}")
    print(f"  Normal Z range: {normals[:, 2].min():.4f} to {normals[:, 2].max():.4f}")
    
    # Verify normals are unit vectors (should be close to 1.0)
    norms = np.linalg.norm(normals, axis=1)
    print(f"  Normal magnitudes: {norms.min():.4f} to {norms.max():.4f} (should be ~1.0)")
    
    # Create output LAS file - MATLAB-compatible format
    # Point Format 3: Has Time, RGB (no NIR, no waveform)
    print(f"\nCreating MATLAB-compatible LAS file: {output_las_path}")
    print("  Format: LAS 1.2, Point Format 3 (XYZ + GPS Time + RGB)")
    
    header = laspy.LasHeader(point_format=3, version="1.2")
    header.offsets = las_in.header.offsets
    header.scales = las_in.header.scales
    
    las_out = laspy.LasData(header)
    
    # Copy all standard point data from input
    las_out.x = las_in.x
    las_out.y = las_in.y
    las_out.z = las_in.z
    
    # Copy intensity if present
    if hasattr(las_in, 'intensity'):
        las_out.intensity = las_in.intensity
        print("  ✓ Copied intensity")
    else:
        las_out.intensity = np.zeros(num_points, dtype=np.uint16)
        print("  ○ Set intensity to 0 (not in input)")
    
    # Copy GPS time if present, otherwise set to 0
    if hasattr(las_in, 'gps_time'):
        las_out.gps_time = las_in.gps_time
        print("  ✓ Copied GPS time")
    else:
        las_out.gps_time = np.zeros(num_points, dtype=np.float64)
        print("  ○ Set GPS time to 0 (not in input)")
    
    # Copy other standard fields
    if hasattr(las_in, 'return_number'):
        las_out.return_number = las_in.return_number
    else:
        las_out.return_number = np.ones(num_points, dtype=np.uint8)
    
    if hasattr(las_in, 'number_of_returns'):
        las_out.number_of_returns = las_in.number_of_returns
    else:
        las_out.number_of_returns = np.ones(num_points, dtype=np.uint8)
    
    if hasattr(las_in, 'classification'):
        las_out.classification = las_in.classification
    else:
        las_out.classification = np.zeros(num_points, dtype=np.uint8)
    
    if hasattr(las_in, 'scan_angle_rank'):
        las_out.scan_angle_rank = las_in.scan_angle_rank
    else:
        las_out.scan_angle_rank = np.zeros(num_points, dtype=np.int8)
    
    if hasattr(las_in, 'user_data'):
        las_out.user_data = las_in.user_data
    else:
        las_out.user_data = np.zeros(num_points, dtype=np.uint8)
    
    if hasattr(las_in, 'point_source_id'):
        las_out.point_source_id = las_in.point_source_id
    else:
        las_out.point_source_id = np.zeros(num_points, dtype=np.uint16)
    
    # Encode normals into RGB channels
    # Normals are in range [-1, 1], convert to [0, 65535]
    # Formula: rgb = (normal + 1) / 2 * 65535
    print("\n  Encoding normals into RGB channels...")
    normals_scaled = ((normals + 1.0) / 2.0 * 65535).astype(np.uint16)
    
    las_out.red = normals_scaled[:, 0]    # normal_x -> Red
    las_out.green = normals_scaled[:, 1]  # normal_y -> Green  
    las_out.blue = normals_scaled[:, 2]   # normal_z -> Blue
    
    print("  ✓ Normals encoded in RGB:")
    print("     normal_x -> Red channel")
    print("     normal_y -> Green channel")
    print("     normal_z -> Blue channel")
    
    # Write the output file
    try:
        las_out.write(output_las_path)
        print(f"\n{'='*70}")
        print("NORMAL ESTIMATION COMPLETED SUCCESSFULLY!")
        print(f"{'='*70}")
        print(f"Input:  {input_las_path}")
        print(f"Output: {output_las_path}")
        print(f"Points: {num_points:,}")
        print(f"Format: LAS 1.2, Point Format 3 (MATLAB-compatible)")
        print(f"\nNormals stored in RGB channels - MATLAB can read these natively!")
        print(f"\nIn MATLAB, extract normals with:")
        print(f"  pc = lasFileReader('{os.path.basename(output_las_path)}');")
        print(f"  ptCloud = readPointCloud(pc);")
        print(f"  % Extract normals from RGB:")
        print(f"  normals_x = (double(ptCloud.Color(:,1)) / 65535) * 2 - 1;")
        print(f"  normals_y = (double(ptCloud.Color(:,2)) / 65535) * 2 - 1;")
        print(f"  normals_z = (double(ptCloud.Color(:,3)) / 65535) * 2 - 1;")
        print(f"  normals = [normals_x, normals_y, normals_z];")
        print(f"{'='*70}")
        return True
    except Exception as e:
        print(f"Error writing output LAS file: {e}")
        import traceback
        traceback.print_exc()
        messagebox.showerror("Error", f"Could not write output LAS file:\n{e}")
        return False


def get_search_radius(points):
    """
    Automatically estimate a good search radius based on point cloud density.
    
    Args:
        points: Numpy array of XYZ coordinates
    
    Returns:
        float: Suggested search radius
    """
    # Sample a subset of points to estimate density
    sample_size = min(10000, len(points))
    sample_indices = np.random.choice(len(points), sample_size, replace=False)
    sample_points = points[sample_indices]
    
    # Build KD-tree and find average distance to 10 nearest neighbors
    pcd_sample = o3d.geometry.PointCloud()
    pcd_sample.points = o3d.utility.Vector3dVector(sample_points.astype(np.float64))
    
    kdtree = o3d.geometry.KDTreeFlann(pcd_sample)
    
    distances = []
    for i in range(min(1000, sample_size)):
        [_, idx, dist] = kdtree.search_knn_vector_3d(sample_points[i], 10)
        if len(dist) > 1:
            distances.append(np.mean(np.sqrt(dist[1:])))  # Skip first (self)
    
    avg_distance = np.mean(distances)
    suggested_radius = avg_distance * 3  # Use 3x average neighbor distance
    
    return suggested_radius


def main():
    """Main function with GUI file selection."""
    print("=" * 70)
    print("LAS Normal Estimation Tool - MATLAB Compatible")
    print("(Stores normals in RGB channels for MATLAB compatibility)")
    print("=" * 70)
    print()
    
    # Select input file
    print("Please select a LAS file for normal estimation...")
    input_path = select_input_file()
    
    if not input_path:
        print("No file selected. Exiting.")
        input("Press Enter to exit...")
        return
    
    print(f"Selected: {input_path}")
    print()
    
    # Generate default output filename
    base_name = os.path.splitext(os.path.basename(input_path))[0]
    default_output = f"{base_name}_with_normals_matlab.las"
    
    # Select output file
    print("Please select where to save the MATLAB-compatible LAS file...")
    output_path = select_output_file(default_output)
    
    if not output_path:
        print("No output location selected. Exiting.")
        input("Press Enter to exit...")
        return
    
    print(f"Output: {output_path}")
    print()
    
    # Load points to estimate search radius
    print("Analyzing point cloud to determine optimal parameters...")
    try:
        las_temp = laspy.read(input_path)
        points_temp = np.vstack([las_temp.x, las_temp.y, las_temp.z]).T
        suggested_radius = get_search_radius(points_temp)
        print(f"Suggested search radius: {suggested_radius:.4f}")
        print()
        
        # Ask user if they want to use suggested radius or custom
        response = messagebox.askyesno(
            "Search Radius",
            f"Suggested search radius: {suggested_radius:.4f}\n\n"
            f"This is based on point cloud density.\n\n"
            f"Use this value?\n\n"
            f"(No = use default value of 0.1)"
        )
        
        if response:
            search_radius = suggested_radius
        else:
            search_radius = 0.1
            print("Using default search radius: 0.1")
    except Exception as e:
        print(f"Could not estimate search radius: {e}")
        print("Using default search radius: 0.1")
        search_radius = 0.1
    
    print(f"\nUsing search radius: {search_radius:.4f}")
    print()
    
    # Perform normal estimation
    success = estimate_normals_matlab_compatible(input_path, output_path, search_radius=search_radius)
    
    if success:
        messagebox.showinfo(
            "Success",
            f"Normal estimation completed successfully!\n\n"
            f"Output file:\n{output_path}\n\n"
            f"Format: LAS 1.2, Point Format 3\n"
            f"Normals stored in RGB channels\n\n"
            f"MATLAB can read this natively with lasFileReader!"
        )
    else:
        print()
        print("=" * 70)
        print("NORMAL ESTIMATION FAILED - See errors above")
        print("=" * 70)
    
    input("\nPress Enter to exit...")


if __name__ == "__main__":
    main()
