"""
LAS Point Cloud Normal Estimation using Open3D

This script loads a LAS file, estimates surface normals using Open3D's 
robust normal estimation algorithms, and saves the result back to a LAS file
with normals stored as extra bytes.

Requirements:
    pip install laspy open3d numpy
"""

import numpy as np
import laspy
import open3d as o3d
import argparse
from pathlib import Path


def las_to_open3d(las_file):
    """
    Convert LAS file to Open3D point cloud
    
    Args:
        las_file: laspy.LasData object
        
    Returns:
        o3d.geometry.PointCloud: Open3D point cloud with colors if available
    """
    # Extract XYZ coordinates
    points = np.vstack((las_file.x, las_file.y, las_file.z)).T
    
    # Create Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    
    # Add colors if available
    if hasattr(las_file, 'red') and hasattr(las_file, 'green') and hasattr(las_file, 'blue'):
        colors = np.vstack((las_file.red, las_file.green, las_file.blue)).T
        # Normalize to 0-1 range
        if colors.max() > 1.0:
            colors = colors / 65535.0
        pcd.colors = o3d.utility.Vector3dVector(colors)
    
    return pcd


def estimate_normals(pcd, radius=None, max_nn=30, orient_method='camera'):
    """
    Estimate normals for the point cloud
    
    Args:
        pcd: Open3D point cloud
        radius: Search radius for normal estimation (None = auto-compute)
        max_nn: Maximum number of nearest neighbors to consider
        orient_method: Method to orient normals
            - 'camera': Orient towards camera location (0,0,0)
            - 'tangent': Use tangent plane orientation
            - None: Don't orient (may have inconsistent directions)
            
    Returns:
        o3d.geometry.PointCloud: Point cloud with estimated normals
    """
    print("Estimating normals...")
    
    # Auto-compute radius if not provided
    if radius is None:
        # Compute average nearest neighbor distance
        pcd_tree = o3d.geometry.KDTreeFlann(pcd)
        distances = []
        num_samples = min(1000, len(pcd.points))
        indices = np.random.choice(len(pcd.points), num_samples, replace=False)
        
        for idx in indices:
            [_, idx_nn, dist_nn] = pcd_tree.search_knn_vector_3d(pcd.points[idx], 2)
            if len(dist_nn) > 1:
                distances.append(np.sqrt(dist_nn[1]))
        
        avg_dist = np.mean(distances)
        radius = avg_dist * 2.5  # Use 2.5x average distance as radius
        print(f"Auto-computed search radius: {radius:.4f}")
    
    # Estimate normals using hybrid search (radius + max neighbors)
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=radius,
            max_nn=max_nn
        )
    )
    
    # Orient normals
    if orient_method == 'camera':
        print("Orienting normals towards camera location...")
        pcd.orient_normals_towards_camera_location(camera_location=np.array([0., 0., 0.]))
    elif orient_method == 'tangent':
        print("Orienting normals using tangent plane method...")
        pcd.orient_normals_to_align_with_direction(
            orientation_reference=np.array([0., 0., 1.])
        )
    
    print(f"Estimated {len(pcd.normals)} normals")
    return pcd


def save_las_with_normals(input_las_path, output_las_path, normals):
    """
    Save LAS file with normals added as extra bytes
    
    Args:
        input_las_path: Path to input LAS file
        output_las_path: Path to output LAS file
        normals: Numpy array of normals (N x 3)
    """
    print(f"Saving LAS file with normals to: {output_las_path}")
    
    # Read original LAS file
    las = laspy.read(input_las_path)

    # Copy header safely
    header = las.header.copy()

    # Add extra dims BEFORE creating LasData
    header.add_extra_dim(laspy.ExtraBytesParams(
        name="NormalX",
        type=np.float32
    ))
    header.add_extra_dim(laspy.ExtraBytesParams(
        name="NormalY",
        type=np.float32
    ))
    header.add_extra_dim(laspy.ExtraBytesParams(
        name="NormalZ",
        type=np.float32
    ))

    # Create output with modified header
    las_out = laspy.LasData(header)

    # Copy original point data
    las_out.points = las.points.copy()

    # Assign normals
    las_out.NormalX = normals[:,0].astype(np.float32)
    las_out.NormalY = normals[:,1].astype(np.float32)
    las_out.NormalZ = normals[:,2].astype(np.float32)

    las_out.write(output_las_path)
    print(f"Successfully saved {len(las_out.points)} points with normals")


def estimate_las_normals(input_path, output_path=None, radius=None, max_nn=30, 
                         orient_method='camera', visualize=False):
    """
    Main function to estimate normals for a LAS file
    
    Args:
        input_path: Path to input LAS file
        output_path: Path to output LAS file (None = auto-generate)
        radius: Search radius for normal estimation (None = auto-compute)
        max_nn: Maximum number of nearest neighbors
        orient_method: Normal orientation method ('camera', 'tangent', or None)
        visualize: Whether to visualize the result
        
    Returns:
        str: Path to output file
    """
    input_path = Path(input_path)
    
    # Generate output path if not provided
    if output_path is None:
        output_path = input_path.parent / f"{input_path.stem}_with_normals{input_path.suffix}"
    else:
        output_path = Path(output_path)
    
    print(f"Loading LAS file: {input_path}")
    las = laspy.read(str(input_path))
    print(f"Loaded {len(las.points)} points")
    
    # Convert to Open3D
    pcd = las_to_open3d(las)
    
    # Estimate normals
    pcd = estimate_normals(pcd, radius=radius, max_nn=max_nn, orient_method=orient_method)
    
    # Extract normals as numpy array
    normals = np.asarray(pcd.normals)
    
    # Save with normals
    save_las_with_normals(str(input_path), str(output_path), normals)
    
    # Visualize if requested
    if visualize:
        print("Visualizing point cloud with normals...")
        print("Press Q to close visualization")
        o3d.visualization.draw_geometries(
            [pcd],
            window_name="Point Cloud with Normals",
            point_show_normal=True,
            width=1024,
            height=768
        )
    
    return str(output_path)


def main():
    """Command-line interface"""
    parser = argparse.ArgumentParser(
        description="Estimate surface normals for LAS point cloud files",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic usage with auto-computed parameters
  python estimate_normals.py input.las
  
  # Specify output file
  python estimate_normals.py input.las -o output.las
  
  # Custom search radius and max neighbors
  python estimate_normals.py input.las -r 0.5 -n 50
  
  # Visualize result
  python estimate_normals.py input.las --visualize
  
  # Use tangent plane orientation
  python estimate_normals.py input.las --orient tangent
        """
    )
    
    parser.add_argument('input', type=str, help='Input LAS file path')
    parser.add_argument('-o', '--output', type=str, default=None,
                       help='Output LAS file path (default: input_with_normals.las)')
    parser.add_argument('-r', '--radius', type=float, default=None,
                       help='Search radius for normal estimation (default: auto-compute)')
    parser.add_argument('-n', '--max-neighbors', type=int, default=30,
                       help='Maximum number of nearest neighbors (default: 30)')
    parser.add_argument('--orient', type=str, choices=['camera', 'tangent', 'none'],
                       default='camera',
                       help='Normal orientation method (default: camera)')
    parser.add_argument('-v', '--visualize', action='store_true',
                       help='Visualize the result with Open3D')
    
    args = parser.parse_args()
    
    # Convert 'none' to None
    orient_method = None if args.orient == 'none' else args.orient
    
    # Process the file
    output_file = estimate_las_normals(
        input_path=args.input,
        output_path=args.output,
        radius=args.radius,
        max_nn=args.max_neighbors,
        orient_method=orient_method,
        visualize=args.visualize
    )
    
    print(f"\n✓ Normal estimation complete!")
    print(f"Output file: {output_file}")


if __name__ == "__main__":
    main()
