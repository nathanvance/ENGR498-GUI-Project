#!/usr/bin/env python3
"""
PCD to LAS Converter with GUI File Dialogs

Converts Point Cloud Data (PCD) files to LAS format with intensity support.
Uses file dialogs for easy file selection.

Requires: open3d, laspy, numpy
Install with: pip install open3d laspy numpy
"""

import sys
import os
import numpy as np
from tkinter import Tk, filedialog, messagebox

try:
    import open3d as o3d
    import laspy
except ImportError as e:
    print(f"Error: Missing required library - {e}")
    print("Install dependencies with: pip install open3d laspy numpy")
    input("Press Enter to exit...")
    sys.exit(1)


def select_input_file():
    """Open file dialog to select input PCD file."""
    root = Tk()
    root.withdraw()  # Hide the main window
    root.attributes('-topmost', True)  # Bring dialog to front
    
    file_path = filedialog.askopenfilename(
        title="Select PCD file to convert",
        filetypes=[("PCD files", "*.pcd"), ("All files", "*.*")]
    )
    
    root.destroy()
    return file_path


def select_output_file(default_name):
    """Open file dialog to select output LAS file location."""
    root = Tk()
    root.withdraw()  # Hide the main window
    root.attributes('-topmost', True)  # Bring dialog to front
    
    file_path = filedialog.asksaveasfilename(
        title="Save LAS file as",
        defaultextension=".las",
        initialfile=default_name,
        filetypes=[("LAS files", "*.las"), ("All files", "*.*")]
    )
    
    root.destroy()
    return file_path


def read_pcd_with_intensity(filepath):
    """
    Read PCD file and extract points, colors, normals, and intensity.
    
    Returns:
        tuple: (points, colors, normals, intensity, has_colors, has_normals, has_intensity)
    """
    points = []
    colors = []
    normals = []
    intensity = []
    
    field_names = []
    field_sizes = []
    field_types = []
    data_type = 'ascii'
    point_count = 0
    
    with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
        # Read header
        header_end = False
        while not header_end:
            line = f.readline().strip()
            
            if line.startswith('FIELDS'):
                field_names = line.split()[1:]
                print(f"Found fields: {field_names}")
            elif line.startswith('SIZE'):
                field_sizes = [int(x) for x in line.split()[1:]]
            elif line.startswith('TYPE'):
                field_types = line.split()[1:]
            elif line.startswith('POINTS'):
                point_count = int(line.split()[1])
            elif line.startswith('DATA'):
                data_type = line.split()[1]
                header_end = True
        
        # Check which fields are present
        has_intensity = 'intensity' in field_names
        has_rgb = 'rgb' in field_names
        has_normal = 'normal_x' in field_names and 'normal_y' in field_names and 'normal_z' in field_names
        
        print(f"Point count: {point_count}")
        print(f"Has intensity: {has_intensity}")
        print(f"Has RGB: {has_rgb}")
        print(f"Has normals: {has_normal}")
        
        if data_type.lower() == 'ascii':
            # Read ASCII data
            for line in f:
                line = line.strip()
                if not line:
                    continue
                    
                values = line.split()
                if len(values) < len(field_names):
                    continue
                
                # Create a dictionary of field values
                data_dict = {field_names[i]: values[i] for i in range(len(field_names))}
                
                # Extract XYZ
                x = float(data_dict.get('x', 0))
                y = float(data_dict.get('y', 0))
                z = float(data_dict.get('z', 0))
                points.append([x, y, z])
                
                # Extract intensity if present
                if has_intensity:
                    intens = float(data_dict.get('intensity', 0))
                    intensity.append(intens)
                
                # Extract normals if present
                if has_normal:
                    nx = float(data_dict.get('normal_x', 0))
                    ny = float(data_dict.get('normal_y', 0))
                    nz = float(data_dict.get('normal_z', 0))
                    normals.append([nx, ny, nz])
                
                # Extract RGB if present
                if has_rgb:
                    # RGB is often packed as a single float, need to unpack
                    rgb_value = int(float(data_dict.get('rgb', 0)))
                    r = ((rgb_value >> 16) & 0xFF) / 255.0
                    g = ((rgb_value >> 8) & 0xFF) / 255.0
                    b = (rgb_value & 0xFF) / 255.0
                    colors.append([r, g, b])
        else:
            print(f"Warning: Binary PCD format detected. Falling back to Open3D for XYZ only.")
            # For binary, use Open3D but we'll lose intensity
            pcd = o3d.io.read_point_cloud(filepath)
            points = np.asarray(pcd.points)
            if pcd.has_colors():
                colors = np.asarray(pcd.colors)
            if pcd.has_normals():
                normals = np.asarray(pcd.normals)
            has_intensity = False
    
    # Convert to numpy arrays
    points = np.array(points, dtype=np.float32)
    has_colors = len(colors) > 0
    has_normals = len(normals) > 0
    has_intensity_data = len(intensity) > 0
    
    if has_colors:
        colors = np.array(colors, dtype=np.float32)
    else:
        colors = None
        
    if has_normals:
        normals = np.array(normals, dtype=np.float32)
    else:
        normals = None
        
    if has_intensity_data:
        intensity = np.array(intensity, dtype=np.float32)
    else:
        intensity = None
    
    return points, colors, normals, intensity, has_colors, has_normals, has_intensity_data


def pcd_to_las(input_pcd_path, output_las_path):
    """
    Convert a PCD file to LAS format with intensity support.
    
    Args:
        input_pcd_path: Path to input PCD file
        output_las_path: Path to output LAS file
    """
    print(f"Reading PCD file: {input_pcd_path}")
    print()
    
    # Read the PCD file with all attributes
    try:
        points, colors, normals, intensity, has_colors, has_normals, has_intensity = read_pcd_with_intensity(input_pcd_path)
    except Exception as e:
        print(f"Error reading PCD file: {e}")
        import traceback
        traceback.print_exc()
        messagebox.showerror("Error", f"Could not read PCD file:\n{e}")
        return False
    
    if len(points) == 0:
        print("Error: PCD file contains no points")
        messagebox.showerror("Error", "PCD file contains no points")
        return False
    
    print(f"\nNumber of points: {len(points)}")
    print(f"Has colors: {has_colors}")
    print(f"Has normals: {has_normals}")
    print(f"Has intensity: {has_intensity}")
    
    # Determine point format
    # Point format 2 = XYZ + Intensity + RGB
    # Point format 1 = XYZ + Intensity (no RGB)
    point_format = 2 if has_colors else 1
    
    # Create LAS file
    print(f"\nCreating LAS file: {output_las_path}")
    
    # Create a new LAS file
    header = laspy.LasHeader(point_format=point_format, version="1.2")
    header.offsets = np.min(points, axis=0)
    header.scales = np.array([0.001, 0.001, 0.001])  # 1mm precision
    
    las = laspy.LasData(header)
    
    # Set XYZ coordinates
    las.x = points[:, 0]
    las.y = points[:, 1]
    las.z = points[:, 2]
    
    # Set intensity values
    if has_intensity and intensity is not None:
        # LAS intensity is typically stored as 16-bit unsigned integer
        intensity_min = intensity.min()
        intensity_max = intensity.max()
        
        print(f"Intensity range: {intensity_min:.2f} to {intensity_max:.2f}")
        
        if intensity_max > intensity_min:
            # Normalize to 0-1, then scale to 0-65535
            normalized = (intensity - intensity_min) / (intensity_max - intensity_min)
            las.intensity = (normalized * 65535).astype(np.uint16)
        else:
            # All values are the same
            las.intensity = np.full(len(points), 32768, dtype=np.uint16)
        
        print("Intensity values written to LAS file")
    else:
        # No intensity data found, set to default value
        las.intensity = np.full(len(points), 0, dtype=np.uint16)
        print("No intensity data found - setting to default (0)")
    
    # Add RGB colors if available
    if has_colors and colors is not None:
        # Convert from [0, 1] to [0, 65535] range for LAS format
        las.red = (colors[:, 0] * 65535).astype(np.uint16)
        las.green = (colors[:, 1] * 65535).astype(np.uint16)
        las.blue = (colors[:, 2] * 65535).astype(np.uint16)
        print("RGB colors written to LAS file")
    
    # Note about normals
    if has_normals:
        print("Normal vectors found (not stored in standard LAS format)")
    
    # Write the LAS file
    try:
        las.write(output_las_path)
        print(f"\n{'='*60}")
        print("Conversion completed successfully!")
        print(f"{'='*60}")
        print(f"Input: {input_pcd_path}")
        print(f"Output: {output_las_path}")
        print(f"Points: {len(points):,}")
        print(f"Format: Point format {point_format} ({'XYZ+Intensity+RGB' if point_format == 2 else 'XYZ+Intensity'})")
        return True
    except Exception as e:
        print(f"Error writing LAS file: {e}")
        import traceback
        traceback.print_exc()
        messagebox.showerror("Error", f"Could not write LAS file:\n{e}")
        return False


def main():
    """Main function with GUI file selection."""
    print("=" * 60)
    print("PCD to LAS Converter")
    print("=" * 60)
    print()
    
    # Select input file
    print("Please select a PCD file to convert...")
    input_path = select_input_file()
    
    if not input_path:
        print("No file selected. Exiting.")
        input("Press Enter to exit...")
        return
    
    print(f"Selected: {input_path}")
    print()
    
    # Generate default output filename
    base_name = os.path.splitext(os.path.basename(input_path))[0]
    default_output = f"{base_name}.las"
    
    # Select output file
    print("Please select where to save the LAS file...")
    output_path = select_output_file(default_output)
    
    if not output_path:
        print("No output location selected. Exiting.")
        input("Press Enter to exit...")
        return
    
    print(f"Output: {output_path}")
    print()
    
    # Perform conversion
    success = pcd_to_las(input_path, output_path)
    
    if success:
        messagebox.showinfo("Success", f"Conversion completed successfully!\n\nOutput file:\n{output_path}")
    else:
        print()
        print("=" * 60)
        print("Conversion failed. See errors above.")
        print("=" * 60)
    
    input("\nPress Enter to exit...")


if __name__ == "__main__":
    main()
