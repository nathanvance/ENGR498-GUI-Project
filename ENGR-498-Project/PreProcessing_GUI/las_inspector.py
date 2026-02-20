#!/usr/bin/env python3
"""
LAS File Attribute Inspector

Simple tool to check what attributes/dimensions are present in a LAS file.
"""

import sys
import numpy as np
from tkinter import Tk, filedialog

try:
    import laspy
except ImportError as e:
    print(f"Error: Missing required library - {e}")
    print("Install with: pip install laspy")
    input("Press Enter to exit...")
    sys.exit(1)


def select_file():
    """Open file dialog to select LAS file."""
    root = Tk()
    root.withdraw()
    root.attributes('-topmost', True)
    
    file_path = filedialog.askopenfilename(
        title="Select LAS file to inspect",
        filetypes=[("LAS files", "*.las"), ("All files", "*.*")]
    )
    
    root.destroy()
    return file_path


def inspect_las(filepath):
    """Inspect LAS file and display all attributes."""
    
    print("=" * 70)
    print(f"Inspecting: {filepath}")
    print("=" * 70)
    
    try:
        las = laspy.read(filepath)
    except Exception as e:
        print(f"Error reading LAS file: {e}")
        return
    
    # Basic info
    print(f"\nBASIC INFORMATION:")
    print("-" * 70)
    print(f"  LAS Version: {las.header.version}")
    print(f"  Point Format: {las.point_format.id}")
    print(f"  Number of Points: {len(las.points):,}")
    print(f"  Point Data Record Length: {las.header.point_format.size} bytes")
    
    # Coordinate info
    print(f"\nCOORDINATE RANGES:")
    print("-" * 70)
    print(f"  X: {las.x.min():.3f} to {las.x.max():.3f}")
    print(f"  Y: {las.y.min():.3f} to {las.y.max():.3f}")
    print(f"  Z: {las.z.min():.3f} to {las.z.max():.3f}")
    
    # Standard dimensions
    print(f"\nSTANDARD DIMENSIONS:")
    print("-" * 70)
    standard_dims = las.point_format.dimension_names
    for dim in standard_dims:
        try:
            data = getattr(las, dim.lower())
            if hasattr(data, 'min'):
                print(f"  ✓ {dim}: min={data.min()}, max={data.max()}")
            else:
                print(f"  ✓ {dim}: (present)")
        except:
            print(f"  ✗ {dim}: (not present)")
    
    # Extra dimensions
    print(f"\nEXTRA DIMENSIONS:")
    print("-" * 70)
    
    if hasattr(las.point_format, 'extra_dimension_names'):
        extra_dims = list(las.point_format.extra_dimension_names)  # Convert generator to list
        
        if len(extra_dims) > 0:
            print(f"  Found {len(extra_dims)} extra dimension(s):")
            for dim in extra_dims:
                try:
                    data = getattr(las, dim)
                    if isinstance(data, np.ndarray):
                        print(f"    ✓ {dim}:")
                        print(f"        Type: {data.dtype}")
                        print(f"        Range: {data.min():.6f} to {data.max():.6f}")
                        print(f"        Mean: {data.mean():.6f}")
                        
                        # Show sample values
                        sample_size = min(5, len(data))
                        print(f"        First {sample_size} values: {data[:sample_size]}")
                    else:
                        print(f"    ✓ {dim}: (present)")
                except Exception as e:
                    print(f"    ✗ {dim}: Error reading - {e}")
        else:
            print("  No extra dimensions found")
    else:
        print("  No extra dimensions found")
    
    # Check specifically for normals
    print(f"\nNORMAL VECTOR CHECK:")
    print("-" * 70)
    has_normals = False
    
    if hasattr(las, 'normal_x') and hasattr(las, 'normal_y') and hasattr(las, 'normal_z'):
        print("  ✓ Normal vectors FOUND!")
        
        nx = las.normal_x
        ny = las.normal_y
        nz = las.normal_z
        
        print(f"\n  Normal X:")
        print(f"    Range: {nx.min():.6f} to {nx.max():.6f}")
        print(f"    Mean: {nx.mean():.6f}")
        
        print(f"\n  Normal Y:")
        print(f"    Range: {ny.min():.6f} to {ny.max():.6f}")
        print(f"    Mean: {ny.mean():.6f}")
        
        print(f"\n  Normal Z:")
        print(f"    Range: {nz.min():.6f} to {nz.max():.6f}")
        print(f"    Mean: {nz.mean():.6f}")
        
        # Check if normals are unit vectors
        normals = np.vstack([nx, ny, nz]).T
        magnitudes = np.linalg.norm(normals, axis=1)
        print(f"\n  Normal magnitudes: {magnitudes.min():.6f} to {magnitudes.max():.6f}")
        print(f"    (Should be ~1.0 for unit vectors)")
        
        # Check if all zeros
        if np.all(normals == 0):
            print("\n  ⚠️  WARNING: All normal values are ZERO!")
        else:
            print(f"\n  ✓ Normal vectors contain valid data")
            
            # Show first 5 normal vectors
            print(f"\n  First 5 normal vectors:")
            for i in range(min(5, len(normals))):
                print(f"    Point {i}: [{nx[i]:.4f}, {ny[i]:.4f}, {nz[i]:.4f}]")
        
        has_normals = True
    else:
        print("  ✗ Normal vectors NOT FOUND")
        print("    (Looking for: normal_x, normal_y, normal_z)")
    
    # RGB check
    print(f"\nCOLOR CHECK:")
    print("-" * 70)
    if hasattr(las, 'red') and hasattr(las, 'green') and hasattr(las, 'blue'):
        print("  ✓ RGB colors FOUND")
        print(f"    Red range: {las.red.min()} to {las.red.max()}")
        print(f"    Green range: {las.green.min()} to {las.green.max()}")
        print(f"    Blue range: {las.blue.min()} to {las.blue.max()}")
    else:
        print("  ✗ RGB colors NOT FOUND")
    
    # Intensity check
    print(f"\nINTENSITY CHECK:")
    print("-" * 70)
    if hasattr(las, 'intensity'):
        print("  ✓ Intensity FOUND")
        print(f"    Range: {las.intensity.min()} to {las.intensity.max()}")
        print(f"    Mean: {las.intensity.mean():.2f}")
    else:
        print("  ✗ Intensity NOT FOUND")
    
    print("\n" + "=" * 70)
    
    return has_normals


def main():
    """Main function."""
    print("LAS File Attribute Inspector")
    print("=" * 70)
    print()
    
    file_path = select_file()
    
    if file_path:
        inspect_las(file_path)
    else:
        print("No file selected.")
    
    input("\nPress Enter to exit...")


if __name__ == "__main__":
    main()
